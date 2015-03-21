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

        protected override Response AsyncTask(Command command)
        {
            throw new NotImplementedException();
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
