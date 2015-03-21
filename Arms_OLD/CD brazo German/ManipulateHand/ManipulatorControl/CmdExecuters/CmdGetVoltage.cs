using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    class CmdGetVoltage:AsyncCommandExecuter
    {
        private TaskPlanner taskPlanner;

        public CmdGetVoltage(TaskPlanner tp)
            : base("arms_getvoltage")
        {
            this.taskPlanner = tp;
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
            double vR=0, vL=0;
            TextBoxStreamWriter.DefaultLog.WriteLine("Command GetVoltage received");
            Response resp;

            if (!this.taskPlanner.ArmsGetVoltage(out vL, out vR))
            {
                resp = Response.CreateFromCommand(command, false);
            }
            else
            {
                if (vR > vL) command.Parameters = vL.ToString("00.0");
                else command.Parameters = vR.ToString("00.0");

                resp = Response.CreateFromCommand(command, true);
            }
            TextBoxStreamWriter.DefaultLog.WriteLine("Sending response " + resp.ToString());

            return resp;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
