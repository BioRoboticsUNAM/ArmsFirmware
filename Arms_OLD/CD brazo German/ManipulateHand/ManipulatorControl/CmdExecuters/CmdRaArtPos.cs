using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdRaArtPos : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;
        double dq1, dq2, dq3, dq4, dq5, dq6, dq7;

        public CmdRaArtPos(TaskPlanner taskPlanner)
            : base("ra_artpos")
        {
            this.taskPlanner = taskPlanner;
            SignatureBuilder sb = new SignatureBuilder();
            sb.AddNewFromDelegate(new SevenDoubleParser(this.ParseSevenDoubles));
            this.Signature = sb.GenerateSignature("ra_artpos");
        }

        public override bool ParametersRequired
        {
            get
            {
                return false;
            }
        }

        protected void ParseSevenDoubles(double d1, double d2, double d3, double d4, double d5, double d6, double d7)
        {
            this.dq1 = d1;
            this.dq2 = d2;
            this.dq3 = d3;
            this.dq4 = d4;
            this.dq5 = d5;
            this.dq6 = d6;
            this.dq7 = d7;
        }

        protected override Response AsyncTask(Command command)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaArtPos: Received: " + command.StringToSend);

            bool success = false;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingRightArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaArtPos: Right Arm is busy executing another command");
                    return Response.CreateFromCommand(command, false);
                }
                if (!this.Signature.CallIfMatch(command))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaArtPos: No match found with parameters options");
                    return Response.CreateFromCommand(command, false);
                }

                //this.CommandManager.Busy = true;

                success = this.taskPlanner.RaGoToArticularPosition(this.dq1, this.dq2, this.dq3, this.dq4, this.dq5, this.dq6, this.dq7);

                if (!this.taskPlanner.MovingLeftArm) this.CommandManager.Busy = false;
                return Response.CreateFromCommand(command, success);
            }
            else
            {
                Vector raPos;
                success = this.taskPlanner.RaGetArticularPosition(out raPos);
                string temp = "";
                for (int i = 0; i < 7; i++)
                    temp += raPos[i].ToString("0.0000") + (i == 6 ? "" : " ");
                command.Parameters = temp;
                return Response.CreateFromCommand(command, success);
            }
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
