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

            if (this.taskPlanner.MovingLeftArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaState: Left Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            
            
            this.CommandManager.Busy = true;
			
            //Agregar revisar el estado
            string state = this.taskPlanner.LaState();
            TextBoxStreamWriter.DefaultLog.WriteLine(state);

            
			this.CommandManager.Busy = false;

            command.Parameters = state;
            return Response.CreateFromCommand(command, true);

        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
	}
}
