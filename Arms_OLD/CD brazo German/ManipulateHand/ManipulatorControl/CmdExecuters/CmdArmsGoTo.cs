using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;


namespace ManipulatorControl
{
	class CmdArmsGoTo : AsyncCommandExecuter
	{
		TaskPlanner taskPlanner;

		public CmdArmsGoTo(TaskPlanner taskPlanner)
			: base("arms_goto")
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

			TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsGoTo: Received: " + command.StringToSend);

			bool success = false;

			if (!command.HasParams)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsGoTo: No parameters received");
				return Response.CreateFromCommand(command, success);
			}
			if (this.taskPlanner.MovingRightArm || this.taskPlanner.MovingLeftArm)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsGoTo: Arms busy executing another command");
				return Response.CreateFromCommand(command, false);
			}
			
			this.CommandManager.Busy = true;

			if (command.Parameters == "home")
			{
				this.taskPlanner.LaOpenGripper(10);
				this.taskPlanner.RaOpenGripper(10);
			}

			success = this.taskPlanner.ArmsGoToPredefPos(command.Parameters);

			this.CommandManager.Busy = false;

			return Response.CreateFromCommand(command, success);

		}

		public override void DefaultParameterParser(string[] parameters)
		{
			throw new NotImplementedException();
		}
	}
}

