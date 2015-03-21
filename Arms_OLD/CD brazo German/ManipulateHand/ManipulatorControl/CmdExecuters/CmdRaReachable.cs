using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdRaReachable : SyncCommandExecuter
    {
        TaskPlanner taskPlanner;
        double x, y, z, roll, pitch, yaw, elbow;

        public CmdRaReachable(TaskPlanner taskPlanner)
            : base("ra_reachable")
        {
            this.taskPlanner = taskPlanner;
            SignatureBuilder sb = new SignatureBuilder();
            sb.AddNewFromDelegate(new SevenDoubleParser(this.ParseSevenDoubles));
            this.Signature = sb.GenerateSignature("ra_reachable");
        }

        protected void ParseSevenDoubles(double d1, double d2, double d3, double d4, double d5, double d6, double d7)
        {
            this.x = d1;
            this.y = d2;
            this.z = d3;
            this.roll = d4;
            this.pitch= d5;
            this.yaw = d6;
            this.elbow = d7;
        }

        protected override Response SyncTask(Command command)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaReachable Received: " + command.StringToSend);

            if (command.HasParams)
            {
                if (!this.Signature.CallIfMatch(command))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaReachable: No match found with parameters options");
                    return Response.CreateFromCommand(command, false);
                }

                return Response.CreateFromCommand(command, this.taskPlanner.RaIsReachablePos(this.x, this.y, this.z, this.roll, this.pitch, this.yaw, this.elbow));
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd RaReachable: Parameters required");
                return Response.CreateFromCommand(command, false);
            }
        }
    }
}
