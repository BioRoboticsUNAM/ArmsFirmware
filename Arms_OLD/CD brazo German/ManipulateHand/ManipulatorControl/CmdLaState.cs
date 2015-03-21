using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
	class CmdLaState : AsyncCommandExecuter
	{
		TaskPlanner taskPlanner;

        public CmdLaState(TaskPlanner taskPlanner)
            : base("la_state")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaState: Received: " + command.StringToSend);

            bool success = false;

			
            if (this.taskPlanner.MovingLeftArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaState: Left Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            
			//Agregar revisar el estado
            
			this.CommandManager.Busy = false;
            return Response.CreateFromCommand(command, success);

        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
	}
}
