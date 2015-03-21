using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdRaRelPos : AsyncCommandExecuter
    {
        private TaskPlanner taskPlanner;
        double x, y, z, roll, pitch, yaw, elbow;

        public CmdRaRelPos(TaskPlanner taskPlanner)
            : base("ra_relpos")
        {
            this.taskPlanner = taskPlanner;
            SignatureBuilder sb = new SignatureBuilder();
            sb.AddNewFromDelegate(new SevenDoubleParser(this.ParseSevenDoubles));
            this.Signature = sb.GenerateSignature("ra_relpos");
        }

        protected void ParseSevenDoubles(double d1, double d2, double d3, double d4, double d5, double d6, double d7)
        {
            this.x = d1;
            this.y = d2;
            this.z = d3;
            this.roll = d4;
            this.pitch = d5;
            this.yaw = d6;
            this.elbow = d7;
        }

        protected override Response AsyncTask(Command command)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaRelPos Received: " + command.StringToSend);

            bool succes;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingRightArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaRelPos: Right Arm is busy executing another command");
                    return Response.CreateFromCommand(command, false);
                }
                if (!this.Signature.CallIfMatch(command))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaRelPos: No match found with parameters options");
                    return Response.CreateFromCommand(command, false);
                }

                //this.CommandManager.Busy = true;

                succes = this.taskPlanner.RaGoToRelativeCartesianPos(this.x, this.y, this.z, this.roll, this.pitch, this.yaw, this.elbow);

                if (!this.taskPlanner.MovingLeftArm) this.CommandManager.Busy = false;
                return Response.CreateFromCommand(command, succes);
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaRelPos: Parameters required");
                return Response.CreateFromCommand(command, false);
            }
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
