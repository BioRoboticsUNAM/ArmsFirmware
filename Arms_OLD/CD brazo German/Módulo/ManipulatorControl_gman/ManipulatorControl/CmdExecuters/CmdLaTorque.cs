using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace ManipulatorControl
{
    public class CmdLaTorque : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;

        public CmdLaTorque(TaskPlanner taskPlanner)
            : base("la_torque")
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
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaTorque: Received: " + command.StringToSend);

            bool success = false;
            Response response = null;

            if (command.HasParams)
            {
                string paramLower = command.Parameters.ToLower();
                if (paramLower.Contains("on") || paramLower.Contains("enable") || paramLower.Contains("true"))
                    success = this.taskPlanner.LaTorque(true);
                else if (paramLower.Contains("off") || paramLower.Contains("disable") || paramLower.Contains("false"))
                    success = this.taskPlanner.LaTorque(false);
                else
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("CmdLaTorque: Invalid parameters");
                    success = false;
                }
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("CmdLaTorque: Parameters required");
                success = false;
            }

            response = Response.CreateFromCommand(command, success);
            TextBoxStreamWriter.DefaultLog.WriteLine("CmdLaTorque: Sent response: " + response.StringToSend);
            return response;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
