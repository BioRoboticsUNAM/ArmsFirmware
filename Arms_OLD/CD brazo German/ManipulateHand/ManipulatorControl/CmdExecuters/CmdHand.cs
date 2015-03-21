using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using System.Threading;

namespace ManipulatorControl
{
    class CmdHand : AsyncCommandExecuter
    {
        TaskPlanner tP;
        public CmdHand(TaskPlanner tskPlan)
            : base("ra_hand")
        {
            this.tP = tskPlan;
        }
        protected override Response AsyncTask(Command command)
        {
            bool succes = false;
            Response resp;
            char[] splitters = {' ','&'};

            TextBoxStreamWriter.DefaultLog.WriteLine("Command ra_hand received");
            //Parsear datos
            if (command.HasParams)
            {
                string[] Parameters = command.Parameters.Split(splitters);
                switch (Parameters.Length)
                {
                    case 1:
                        succes = this.tP.RaHand(double.Parse(Parameters[0]), 0, double.Parse(Parameters[0]), double.Parse(Parameters[0]));
                        break;// agarrar
                    case 2:
                        this.tP.RaHand(100,100,100,0);//apuntar
                        Thread.Sleep(500);
                        succes = this.tP.RaHand(double.Parse(Parameters[0]), 15.0, double.Parse(Parameters[1]), 0);
                        break;
                    case 4:
                        succes = this.tP.RaHand(double.Parse(Parameters[0]),double.Parse( Parameters[1]),double.Parse( Parameters[2]),double.Parse( Parameters[3]));
                        break;
                    default:
                        TextBoxStreamWriter.DefaultLog.WriteLine("cmdHand: Incorrect parameters length");
                        break;
                }

            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Command ra_hand: No parameters");
            }
            resp = Response.CreateFromCommand(command, succes);
            return resp;
        }

        public override void DefaultParameterParser(string[] parameters)
        {
            throw new NotImplementedException();
        }

        public override bool ParametersRequired
        {
            get
            {
                return true;
            }
        }
		 //if (command.HasParams)
		 //   {	string position = command.Parameters;
		 //       TextBoxStreamWriter.DefaultLog.WriteLine("Command ra_handMove params=" + position);

		 //       switch(position)
		 //           case "palma":
		 //                   tP.RaHand(
		 //               break;

		 //   }
    }
}
