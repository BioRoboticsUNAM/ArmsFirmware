using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;

namespace Head
{
    class CmdLookAtStop : AsyncCommandExecuter
    {
        private TaskMan taskMan;

        public CmdLookAtStop(TaskMan tM)
            : base("hd_lookatstop")
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

            TextBoxStreamWriter.DefaultLog.WriteLine("Command LookAtStop received " + command.ToString());

            succes = this.taskMan.StopMove();

            resp = Response.CreateFromCommand(command, succes);
            TextBoxStreamWriter.DefaultLog.WriteLine("Sending response " + resp.ToString());
            return resp;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }
    }
}
