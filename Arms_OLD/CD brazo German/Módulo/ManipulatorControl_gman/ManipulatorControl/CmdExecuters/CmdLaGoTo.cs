using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    public class CmdLaGoTo : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdLaGoTo(TaskPlanner taskPlanner)
            : base("la_goto")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaGoTo: Received: " + command.StringToSend);

            bool success = false;

            if (!command.HasParams)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaGoTo: No parameters received");
                return Response.CreateFromCommand(command, success);
            }
            if (this.taskPlanner.MovingLeftArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaGoTo: Left Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            //this.CommandManager.Busy = true;
            if (command.Parameters == "home") this.taskPlanner.LaOpenGripper(1);
            success = this.taskPlanner.LaGoToPredefPos(command.Parameters);

            if (!this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;
            return Response.CreateFromCommand(command, success);

        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
