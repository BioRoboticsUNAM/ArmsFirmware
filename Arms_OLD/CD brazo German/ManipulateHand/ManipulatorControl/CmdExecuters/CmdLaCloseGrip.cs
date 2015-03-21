using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdLaCloseGrip : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdLaCloseGrip(TaskPlanner taskPlanner)
            : base("la_closegrip")
        {
            this.taskPlanner = taskPlanner;
        }

        public override bool ParametersRequired
        {
            get
            {
                return false;
            }
        }

        protected override Response AsyncTask(Command command)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaCloseGrip: Received: " + command.StringToSend);

            bool success = false;
            bool ObjectInHand = false;

            Response response;
            double force;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingLeftArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaCloseGrip: Left Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }

                if (!double.TryParse(command.Parameters, out force))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaCloseGrip: Can't parse parameters");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }

                success = this.taskPlanner.LaCloseGripper(force, out ObjectInHand);
                command.Parameters = ObjectInHand.ToString();

                if (!this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;
                return Response.CreateFromCommand(command, success);
            }
            else
            {
                if (this.taskPlanner.MovingLeftArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaCloseGrip: Left Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }

                success = this.taskPlanner.LaCloseGripper(30, out ObjectInHand);
                command.Parameters = ObjectInHand.ToString();

                if (!this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;
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
