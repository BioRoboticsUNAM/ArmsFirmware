using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    public class CmdRaTorque : AsyncCommandExecuter
    {
        private TaskPlanner taskPlanner;

        public CmdRaTorque(TaskPlanner taskPlanner)
            : base("ra_torque")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaTorque: Received: " + command.StringToSend);

            bool success = false;
            Response response = null;

            if (command.HasParams)
            {
                string paramLower = command.Parameters.ToLower();
                if (paramLower.Contains("on") || paramLower.Contains("enable") || paramLower.Contains("true"))
                    success = this.taskPlanner.RaTorque(true);
                else if (paramLower.Contains("off") || paramLower.Contains("disable") || paramLower.Contains("false"))
                    success = this.taskPlanner.RaTorque(false);
                else
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaTorque: Invalid parameters");
                    success = false;
                }
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaTorque: Parameters required");
                success = false;
            }

            response = Response.CreateFromCommand(command, success);
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaTorque: Sent response: " + response.StringToSend);
            return response;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
