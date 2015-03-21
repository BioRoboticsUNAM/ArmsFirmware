using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Robotics.API;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class CmdLaReachable : SyncCommandExecuter
    {
        TaskPlanner taskPlanner;
        double x, y, z, roll, pitch, yaw, elbow;

        public CmdLaReachable(TaskPlanner taskPlanner)
            : base("la_reachable")
        {
            this.taskPlanner = taskPlanner;
            SignatureBuilder sb = new SignatureBuilder();
            sb.AddNewFromDelegate(new SevenDoubleParser(this.ParseSevenDoubles));
            this.Signature = sb.GenerateSignature("la_reachable");
        }

        protected void ParseSevenDoubles(double d1, double d2, double d3, double d4, double d5, double d6, double d7)
        {
            this.x = d1;
            this.y = d2;
            this.z = d3;
            this.roll = d4;
            this.pitch = d5;
            this.yaw = d6;
            this.elbow = d7;
        }

        protected override Response SyncTask(Command command)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaReachable Received: " + command.StringToSend);

            if (command.HasParams)
            {
                if (!this.Signature.CallIfMatch(command))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaReachable: No match found with parameters options");
                    return Response.CreateFromCommand(command, false);
                }

                return Response.CreateFromCommand(command, this.taskPlanner.LaIsReachablePos(this.x, this.y, this.z, this.roll, this.pitch, this.yaw, this.elbow));
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Cmd LaReachable: Parameters required");
                return Response.CreateFromCommand(command, false);
            }
        }
    }
}
