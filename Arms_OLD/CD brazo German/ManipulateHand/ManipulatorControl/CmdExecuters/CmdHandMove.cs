using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using System.Threading;

namespace ManipulatorControl
{
    class CmdHandMove : AsyncCommandExecuter
    {
        TaskPlanner taskPlanner;
         public CmdHandMove (TaskPlanner tskPlan) 
             : base("ra_hand_move")
        {
            this.taskPlanner = tskPlan;
        }

        protected override Response AsyncTask(Command command)
        {
            bool succes = false;
            Response resp;

            TextBoxStreamWriter.DefaultLog.WriteLine("Command ra_handmove received");
            if (command.HasParams)
            {
                string position = command.Parameters;
                TextBoxStreamWriter.DefaultLog.WriteLine("Command ra_hand_move params=" + position);
                switch (position)
                {
                    case "greeting":
                        succes = this.taskPlanner.RaHand(50, 80, 100, 100);
                        break;
                    case "fist":
                        this.taskPlanner.RaHand(80, 80, 0, 0);
                        Thread.Sleep(800);
                        succes = this.taskPlanner.RaHand(10, 30, 0, 0);
                        break;
                    case "handshake":
                        succes = this.taskPlanner.RaHand(10, 0, 95, 95);
                        break;
                    case "handshake1":
                        succes = this.taskPlanner.RaHand(0, 0, 50, 50);
                        break;
                    case "point":
                        succes = this.taskPlanner.RaHand(80, 80, 100, 0);
                        break;
                    case "perfect":
                        this.taskPlanner.RaHand(80, 80, 10, 100);
                        Thread.Sleep(500);
                        succes = this.taskPlanner.RaHand(15, 10, 10, 100);
                        break;
                    case "good":
                        succes = this.taskPlanner.RaHand(70, 80, 0, 0);
                        break;
                    case "mark":
                        this.taskPlanner.RaHand(80, 80, 0, 0);
                        succes = this.taskPlanner.RaHand(10, 20, 100, 0);
                        break;

                    default:
                        TextBoxStreamWriter.DefaultLog.WriteLine("cmdHand: Incorrect phrase");
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
    }
}
