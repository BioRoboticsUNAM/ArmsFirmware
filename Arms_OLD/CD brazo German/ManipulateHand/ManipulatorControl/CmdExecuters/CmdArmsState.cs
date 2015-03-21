using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;


namespace ManipulatorControl
{
    class CmdArmsState : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdArmsState(TaskPlanner taskPlanner)
            : base("arms_state")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsState: Received: " + command.StringToSend);

            bool success = false;
            this.CommandManager.Busy = true;

            string states = "";

            if (this.taskPlanner.MovingRightArm || this.taskPlanner.MovingLeftArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd ArmsState: Arms busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            
            this.CommandManager.Busy = true;

            states += this.taskPlanner.RaState() + " ";
            states += this.taskPlanner.LaState();

            TextBoxStreamWriter.DefaultLog.WriteLine(states);
            
            this.CommandManager.Busy = false;

            command.Parameters = states;

            return Response.CreateFromCommand(command, true);

        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}