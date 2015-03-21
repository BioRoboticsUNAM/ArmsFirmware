using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdLaAbsPos : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;
        private int paramsCounter;
        private double x, y, z, roll, pitch, yaw, elbow;

        public CmdLaAbsPos(TaskPlanner taskPlanner)
            : base("la_abspos")
        {
            this.taskPlanner = taskPlanner;
            SignatureBuilder sb = new SignatureBuilder();
            sb.AddNewFromDelegate(new ThreeDoubleParser(this.ParseThreeDoubles));
            sb.AddNewFromDelegate(new SixDoubleParser(this.ParseSixDoubles));
            sb.AddNewFromDelegate(new SevenDoubleParser(this.ParseSevenDoubles));
            this.Signature = sb.GenerateSignature("la_abspos");
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Received: " + command.StringToSend);

            bool success = false;
			Response response;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingLeftArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Left Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }
                if (!this.Signature.CallIfMatch(command))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: No match found with parameters options");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }
                //this.CommandManager.Busy = true;
                if (this.paramsCounter == 7)
                    success = this.taskPlanner.LaGoToCartesianPosition(x, y, z, roll, pitch, yaw, elbow);
                else if (this.paramsCounter == 6)
                    success = this.taskPlanner.LaGoToCartesianPosition(x, y, z, roll, pitch, yaw);
                else if (this.paramsCounter == 3)
                    success = this.taskPlanner.LaGoToCartesianPosition(x, y, z);
                else
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: No match found with parameters options");

                if(!this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
				return response;
            }
            else
            {
                Vector laPos;
                success = this.taskPlanner.LaGetCartesianPosition(out laPos);
                string temp = "";
                for (int i = 0; i < 7; i++)
                    temp += laPos[i].ToString("0.000") + (i == 6 ? "" : " ");
                command.Parameters = temp;
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
				return response;
            }
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
