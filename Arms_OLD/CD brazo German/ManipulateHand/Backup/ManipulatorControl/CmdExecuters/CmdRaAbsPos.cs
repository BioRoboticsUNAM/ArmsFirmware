using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdRaAbsPos : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;
        private int paramsCounter;
        private double x, y, z, roll, pitch, yaw, elbow;

        public CmdRaAbsPos(TaskPlanner taskPlanner)
            : base("ra_abspos")
        {
            this.taskPlanner = taskPlanner;
            SignatureBuilder sb = new SignatureBuilder();
            sb.AddNewFromDelegate(new ThreeDoubleParser(this.ParseThreeDoubles));
            sb.AddNewFromDelegate(new SixDoubleParser(this.ParseSixDoubles));
            sb.AddNewFromDelegate(new SevenDoubleParser(this.ParseSevenDoubles));
            this.Signature = sb.GenerateSignature("ra_abspos");
        }

        public override bool ParametersRequired
        {
            get
            {
                return false;
            }
        }

        protected void ParseThreeDoubles(double d1, double d2, double d3)
        {
            this.x = d1;
            this.y = d2;
            this.z = d3;
            this.paramsCounter = 3;
        }

        protected void ParseSixDoubles(double d1, double d2, double d3, double d4, double d5, double d6)
        {
            this.x = d1;
            this.y = d2;
            this.z = d3;
            this.roll = d4;
            this.pitch = d5;
            this.yaw = d6;
            this.paramsCounter = 6;
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
            this.paramsCounter = 7;
        }

        protected override Response AsyncTask(Command command)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: Received: " + command.StringToSend);

            bool success = false;
			Response response;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingRightArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: Right Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }
                if (!this.Signature.CallIfMatch(command))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: No match found with parameters options");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }
                //this.CommandManager.Busy = true;
                if (this.paramsCounter == 7)
                    success = this.taskPlanner.RaGoToCartesianPosition(x, y, z, roll, pitch, yaw, elbow);
                else if (this.paramsCounter == 6)
                    success = this.taskPlanner.RaGoToCartesianPosition(x, y, z, roll, pitch, yaw);
                else if (this.paramsCounter == 3)
                    success = this.taskPlanner.RaGoToCartesianPosition(x, y, z);
                else
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: No match found with parameters options");

                if (!this.taskPlanner.MovingLeftArm) this.CommandManager.Busy = false;
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: Sent response: " + response.StringToSend);
				return response;
            }
            else
            {
                Vector raPos;
                success = this.taskPlanner.RaGetCartesianPosition(out raPos);
                string temp = "";
                for (int i = 0; i < 7; i++)
                    temp += raPos[i].ToString("0.000") + (i == 6 ? "" : " ");
                command.Parameters = temp;
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaAbsPos: Sent response: " + response.StringToSend);
				return response;
            }
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
