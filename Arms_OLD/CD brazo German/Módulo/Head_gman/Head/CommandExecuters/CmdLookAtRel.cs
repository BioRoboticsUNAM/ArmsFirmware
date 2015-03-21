using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.Controls;
using Robotics.API;

namespace Head
{
    class CmdLookAtRel: AsyncCommandExecuter
    {
        private TaskMan taskMan;

        public CmdLookAtRel(TaskMan tM)
            : base("hd_lookatrel")
        {
            this.taskMan = tM;
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
            Response resp;
            bool succes = false;
            string[] parameters;
            char[] delimiters = { ' ', ',' };

            TextBoxStreamWriter.DefaultLog.WriteLine("Command LookAtRel received " + command.ToString());

            parameters = command.Parameters.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);

             if (parameters.Length == 2)
            {
                succes = this.taskMan.SetPosRel(parameters[0], parameters[1]);
            }
             else
             {
                 TextBoxStreamWriter.DefaultLog.WriteLine("CmdLookAtRel: Incorrect number of parameters");
             }


            resp = Response.CreateFromCommand(command, succes);
            TextBoxStreamWriter.DefaultLog.WriteLine("Sending Response " + resp.ToString());

            return resp;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
