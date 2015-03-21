using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdLaOpenGripper : AsyncCommandExecuter
    {
        private TaskPlanner taskPlanner;

        public CmdLaOpenGripper(TaskPlanner taskPlanner)
            : base("la_opengrip")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaOpenGrip: Received: " + command.StringToSend);

            bool success = false;
			Response response;
            double perc;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingLeftArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaOpenGrip: Left Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }
                if (!double.TryParse(command.Parameters, out perc))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaOpenGrip: Can't parse parameters");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
					return response;
                }

                success = this.taskPlanner.LaOpenGripper(perc);

                if (!this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaAbsPos: Sent response: " + response.StringToSend);
				return response;
            }
            else
            {
                if (this.taskPlanner.MovingLeftArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaOpenGrip: Left Arm is busy executing another command");
                    return Response.CreateFromCommand(command, false);
                }

                success = this.taskPlanner.LaOpenGripper(80);

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
