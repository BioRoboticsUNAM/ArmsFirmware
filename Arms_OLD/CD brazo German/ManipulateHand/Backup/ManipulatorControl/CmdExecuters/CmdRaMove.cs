using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
	class CmdRaMove : AsyncCommandExecuter
	{
		TaskPlanner taskPlanner;

        public CmdRaMove(TaskPlanner taskPlanner)
            : base("ra_move")
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
			TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaMove: Received: " + command.StringToSend);

			bool success = false;

			if (!command.HasParams)
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaMove: No parameters received");
				return Response.CreateFromCommand(command, success);
			}

            if (this.taskPlanner.MovingRightArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaMove: Right Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            //this.CommandManager.Busy = true;

			success = this.taskPlanner.RaPerformMovement(command.Parameters);

            if (!this.taskPlanner.MovingLeftArm) this.CommandManager.Busy = false;
			return Response.CreateFromCommand(command, success);

		}

		public override void DefaultParameterParser(string[] parameters)
		{
			throw new NotImplementedException();
		}
	}
}
