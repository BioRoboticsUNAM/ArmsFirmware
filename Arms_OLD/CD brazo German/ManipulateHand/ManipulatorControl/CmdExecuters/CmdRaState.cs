using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    class CmdRaState : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdRaState(TaskPlanner taskPlanner)
            : base("ra_state")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaState: Received: " + command.StringToSend);

            if (this.taskPlanner.MovingRightArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaState: Right Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            
            this.CommandManager.Busy = true;

            //Agregar revisar el estado
            string state = this.taskPlanner.RaState();
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
