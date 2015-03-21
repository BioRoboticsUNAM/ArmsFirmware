using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;



namespace Head
{
    class CmdLookAt: AsyncCommandExecuter
    {
        private TaskMan taskMan;

        public CmdLookAt(TaskMan tM)
            : base("hd_lookat")
        {
            this.taskMan = tM;
        }

    
        protected override Response AsyncTask(Command command)
        {
            Response resp;
            bool succes = false;
            string position;
            string[] parameters;
            char[] delimiters = {' ',','};

            TextBoxStreamWriter.DefaultLog.WriteLine("Command LookAt received " + command.ToString());

            if (command.Parameters == "")
            {
                succes = this.taskMan.GetPosition(out position);
                command.Parameters = position;
            }
            else
            {
                parameters = command.Parameters.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);
                
                if (parameters.Length == 2)
                {
                    succes = this.taskMan.SetPosition(parameters[0], parameters[1]);
                }
                else if (parameters.Length == 3)
                {
                    succes = this.taskMan.SetPosSp(parameters[0], parameters[1], parameters[2]);
                }
                else
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("CmdLookAt: Incorrect number of parameters");
                }
            }

            resp = Response.CreateFromCommand(command,succes);
            TextBoxStreamWriter.DefaultLog.WriteLine("Sending Response " + resp.ToString());
            
            return resp;
        }

        public override bool ParametersRequired
        {
            get
            {
                return false;
            }
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
