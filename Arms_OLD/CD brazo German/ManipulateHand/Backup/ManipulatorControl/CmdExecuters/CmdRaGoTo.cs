using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    public class CmdRaGoTo : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdRaGoTo(TaskPlanner taskPlanner)
            : base("ra_goto")
        {
            this.taskPlanner = taskPlanner;
        }

		public override bool ParametersRequired
		{
			get
			{
				return true;
			}
		}



        protected override Response AsyncTask(Command command)
        {
			TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaGoTo: Received: " + command.StringToSend);
			
			bool success = false;

			if (!command.HasParams)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaGoTo: No parameters received");
				return Response.CreateFromCommand(command, success);
			}
            if (this.taskPlanner.MovingRightArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaGoTo: Right Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            //this.CommandManager.Busy = true;
            
            if (command.Parameters == "home") this.taskPlanner.RaOpenGripper(10);
			success = this.taskPlanner.RaGoToPredefPos(command.Parameters);

            if (!this.taskPlanner.MovingLeftArm) this.CommandManager.Busy = false;
			return Response.CreateFromCommand(command, success);

        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
