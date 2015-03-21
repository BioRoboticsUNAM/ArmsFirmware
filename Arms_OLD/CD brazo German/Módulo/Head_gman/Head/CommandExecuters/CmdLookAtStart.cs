using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.Controls;
using Robotics.API;

namespace Head
{
    class CmdLookAtStart : AsyncCommandExecuter
    {
        private TaskMan taskMan;
        public CmdLookAtStart(TaskMan tM)
            : base("hd_lookatstart")
        {
            this.taskMan = tM;
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
            Response resp;
            bool succes = false;
            string[] parameters;
            char[] delimiters = {',',' '};

            TextBoxStreamWriter.DefaultLog.WriteLine("Command LookAtStart received " + command.ToString());
            
            parameters = command.Parameters.Split(delimiters,StringSplitOptions.RemoveEmptyEntries);
            if (parameters.Length == 2)
                succes = this.taskMan.StartMove(parameters[0], parameters[1]);
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("CmdLookAtStart: Incorrect number of parameters");
            }

            resp = Response.CreateFromCommand(command, succes);
            TextBoxStreamWriter.DefaultLog.WriteLine("Sending response "+ resp.ToString());
            return resp;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
