using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
	class CmdArmsMove : AsyncCommandExecuter
	{
		TaskPlanner taskPlanner;

		public CmdArmsMove(TaskPlanner taskPlanner)
			: base("arms_move")
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
			TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsMove: Received: " + command.StringToSend);

			bool success = false;

			if (!command.HasParams)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsMove: No parameters received");
				return Response.CreateFromCommand(command, success);
			}
			if (this.taskPlanner.MovingRightArm || this.taskPlanner.MovingLeftArm)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsMove: Arms busy executing another command");
				return Response.CreateFromCommand(command, false);
			}
			//this.CommandManager.Busy = true;

			success = this.taskPlanner.ArmsMove(command.Parameters);

			if (!this.taskPlanner.MovingLeftArm && !this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;

			return Response.CreateFromCommand(command, success);

		}

		public override void DefaultParameterParser(string[] parameters)
		{
			throw new NotImplementedException();
		}
	}

	
}
