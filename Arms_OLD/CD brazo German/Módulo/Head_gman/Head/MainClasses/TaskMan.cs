using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Robotics.API;
using Robotics.Controls;

namespace Head
{
    public class TaskMan
    {
        
        private Settings settings;
        private HeadStatus status;
        private CommandManager cmdMan;
        private Thread slowMove;
        public Head head;

        public TaskMan(HeadManager hM)
        {
            this.head = new Head(hM);
            this.status = hM.status;
            this.settings = hM.settings;
            this.cmdMan = hM.cmdMan;
            TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Succesfully started");
        }

        public bool StopMove()
        {
            try
            {
                this.status.Move = false;
                Thread.Sleep(20);
                if (this.slowMove.IsAlive) this.slowMove.Join();
                if (this.slowMove.IsAlive) this.slowMove.Abort();
                return true;
            }
            catch
            {

                return false;
            }
        }
        public bool StartMove(string pan, string tilt)
        {
            double Pan = 0;
            double Tilt = 0;


            if (this.status.IsMoving)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Head is busy");
                return false;
            }

            if (!double.TryParse(pan, out Pan) || !double.TryParse(tilt, out Tilt))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't parse parameters");
                return false;
            }

            this.slowMove = new Thread(new ThreadStart(slowMoveTask));
            this.slowMove.IsBackground = true;
            this.status.Move = true;
            this.slowMove.Start();

            return true;
        }

        void slowMoveTask()
        {
            Head smHead = this.head;
            CommandManager smMan = this.cmdMan;
            HeadStatus smStatus = this.status;

            double Pan = this.status.GlobalPan;
            double Tilt = this.status.GlobalTilt;

            double delta = 0.1;
            int signoP,signoT;
            
            if(smStatus.smPan>Pan) signoP = 1;
            else signoP = -1;

            if (smStatus.smTilt > Tilt) signoT = 1;
            else signoT = -1;

            while (smStatus.Move)
            {
                Pan += delta*signoP;
                Tilt += delta*signoT;

                if (Math.Abs(Pan) > Math.Abs(smStatus.smPan)) Pan = smStatus.smPan;
                if (Math.Abs(Tilt) > Math.Abs(smStatus.smTilt)) Tilt = smStatus.smTilt;

                smHead.LookAt(Pan, Tilt);

                if (Math.Abs(Pan) == Math.Abs(smStatus.smPan) && Math.Abs(Tilt) == Math.Abs(smStatus.smTilt)) smStatus.Move = false;
            }

        }

        public bool GetPosition(out string pos)
        {
            pos = "";
            if (this.status.IsMoving)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Head is busy");
                return false;
            }

            if (!this.head.GetLookAtPosition(out pos))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't get position");
                return false;
            }

            TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Looking at position " + pos);
            return true;
        }

        public bool SetPosition(string pan, string tilt)
        {
            bool succes = false;
            double Tilt = 0, Pan = 0;

            if (this.status.IsMoving)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Head is busy");
                return false;
            }

            if (!double.TryParse(pan, out Pan) || !double.TryParse(tilt, out Tilt))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't parse parameters");
                return succes;
            }

            succes = this.head.LookAt(Pan, Tilt);
            if (!succes)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't set LookAtPosition");
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Succesful LookAtPosition " + Pan.ToString("0.00") + " " + Tilt.ToString("0.00"));

            }

            return succes;
        }

        public bool SetPosSp(string pan, string tilt, string speed)
        {
            bool succes = false;
            double Tilt = 0, Pan = 0, Speed = 1;

            if (this.status.IsMoving)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Head is busy");
                return false;
            }

            if (!double.TryParse(pan, out Pan) || !double.TryParse(tilt, out Tilt) || !double.TryParse(speed,out Speed))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't parse parameters");
                return succes;
            }

            if (Speed == 0) Speed = 1;

            succes = this.head.LookAt(Pan, Tilt, Speed);
            
            if (!succes)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't set LookAtPosition with desired SpeedPercentage");
            }
            else TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Succesful LookAtPositionSpeed " + Pan.ToString("0.00") + " " + Tilt.ToString("0.00") + " " + Speed);

            return succes;
        }

        public bool SetPosRel(string rel_pan, string rel_tilt)
        {
            bool succes = false;
            double r_tilt = 0, r_pan = 0;

            if (this.status.IsMoving)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Head is busy");
                return false;
            }

            if (!double.TryParse(rel_pan, out r_pan) || !double.TryParse(rel_tilt, out r_tilt))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't parse parameters");
                return succes;
            }

            succes = this.head.LookAtRel(r_pan, r_tilt);
            if (!succes)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Can't set LookAtRelPosition");
            }
            else TextBoxStreamWriter.DefaultLog.WriteLine("TaskManager: Succesful LookAtRelPosition " + r_pan.ToString("0.00") + " " + r_tilt.ToString("0.00") + " Absolute position " + this.status.GlobalPan + " " + this.status.GlobalTilt);

            return succes;
        }
        //Agregar los métodos a jalar por los comandos, agregar clases de los comandos
    }
}
