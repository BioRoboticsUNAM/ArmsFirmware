using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    class CmdLaMove : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdLaMove(TaskPlanner taskPlanner)
            : base("la_move")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaMove: Received: " + command.StringToSend);

            bool success = false;

            if (!command.HasParams)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaMove: No parameters received");
                return Response.CreateFromCommand(command, success);
            }

            if (this.taskPlanner.MovingLeftArm)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaMove: Left Arm is busy executing another command");
                return Response.CreateFromCommand(command, false);
            }
            //this.CommandManager.Busy = true;

            success = this.taskPlanner.LaPerformMovement(command.Parameters);

            if (!this.taskPlanner.MovingRightArm) this.CommandManager.Busy = false;
            return Response.CreateFromCommand(command, success);

        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
