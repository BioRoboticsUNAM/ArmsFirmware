using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdRaOpenGripper : AsyncCommandExecuter
    {
        private TaskPlanner taskPlanner;

        public CmdRaOpenGripper(TaskPlanner taskPlanner)
            : base("ra_opengrip")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Received: " + command.StringToSend);

            bool success = false;
			Response response;
            double perc;

            if (command.HasParams)
            {
                if (this.taskPlanner.MovingRightArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Right Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Sent response: " + response.StringToSend);
					return response;
                }

                if (!double.TryParse(command.Parameters, out perc))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Can't parse parameters");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Sent response: " + response.StringToSend);
					return response;
                }

				success = this.taskPlanner.RaOpenGripper(perc);              

                if (!this.taskPlanner.MovingLeftArm) 
					this.CommandManager.Busy = false;
				
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Sent response: " + response.StringToSend);
				return response;
            }
            else
            {
                if (this.taskPlanner.MovingRightArm)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Right Arm is busy executing another command");
					response = Response.CreateFromCommand(command, false);
					TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Sent response: " + response.StringToSend);
					return response;
                }

				success = this.taskPlanner.RaOpenGripper(80);
                
                if (!this.taskPlanner.MovingLeftArm) 
					this.CommandManager.Busy = false;
				response = Response.CreateFromCommand(command, success);
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaOpenGrip: Sent response: " + response.StringToSend);
				return response;
            }
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
