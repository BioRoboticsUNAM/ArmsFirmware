using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading;
using Robotics.Mathematics;
using Robotics.Controls;
using System.Diagnostics;

namespace Head
{
    public class Head
    {

        #region Variables

        SerialPortManager serialPortManager;
        public ServoControl[] servo;
        private Settings settings;
        private HeadStatus status;
       
        private HeadManager headManager;

        #endregion
        #region Constructor
        public Head(HeadManager hM)
        {
            this.headManager = hM;
            this.settings = this.headManager.settings;
            this.status = this.headManager.status;
            
            //Settings
            serialPortManager = new SerialPortManager(this.settings);
            servo = new ServoControl[2];
            servo[0] = new ServoControl(this.settings.servoPan, this.settings.idpan, this.settings.zeroPan, false, serialPortManager);
            servo[1] = new ServoControl(this.settings.servoTilt, this.settings.idtilt, this.settings.zeroTilt, false, serialPortManager);

            this.status.HeadIsReady = InicializedHead();
        }
        #endregion

        public bool InicializedHead()
        {
            for (int i = 0; i < servo.Length; i++)
            {
                if (!servo[i].SetSpeedPerc(this.settings.maxSpeedPerc))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head: Couldn't set Max Speeds");
                    return false;
                }
                
                Thread.Sleep(20);
            }

            for (int j = 0; j < servo.Length; j++)
            {
                if (!servo[j].TorqueEnable(1))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head: Couldn't enable torque");
                    return false;
                }
                Thread.Sleep(20);
            }

            for (int j = 0; j < servo.Length; j++)
            {
                if (!servo[j].SetPosition(0))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head: Couldn't set initial position 0, 0");
                    return false;
                }
                Thread.Sleep(20);
            }

            for (int j = 0; j < servo.Length; j++)
            {
                if (!servo[j].SetCWComplianceSlope(128))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head: Couldn't set CW Compliance slope");
                    return false;
                }
                Thread.Sleep(20);
            }
            for (int j = 0; j < servo.Length; j++)
            {
                if (!servo[j].SetCCWComplianceSlope(128))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head: Couldn't set CCW Compliance slope");
                    return false;
                }
                Thread.Sleep(20);
            }

            TextBoxStreamWriter.DefaultLog.WriteLine("Head: Initialization completed succesfully...");
            return true;
        }

        #region Usable on TaskManager
        public bool LookAt(double pan, double tilt)
        {
            return LookAtCalc(pan, tilt, this.settings.maxSpeedPerc);
        }

        public bool LookAt(double pan, double tilt, double speed)
        {
            return LookAtCalc(pan, tilt, speed);
        }

        public bool LookAtRel(double rel_pan, double rel_tilt)
        {

            double pan, tilt;
            pan = this.status.GlobalPan + rel_pan;
            tilt = this.status.GlobalTilt + rel_tilt;

            return LookAt(pan, tilt);
        }
        
        public bool GetLookAtPosition(out string position)
        {
            position = "";
            double pan, tilt;
            if (!GetLookAtPosition(out pan, out tilt)) return false;
            position = pan.ToString("0.00") + " " + tilt.ToString("0.00");
            return true;
        }
        #endregion

        #region Calc
        public bool LookAtCalc(double pan, double tilt, double speed)
        {
            if (pan < -Math.PI / 2) pan = -Math.PI / 2;
            if (pan > Math.PI / 2) pan = Math.PI / 2;

            if (tilt < -1.2) tilt = -1.2;
            if (tilt > 1.2) tilt = 1.2;

            if (speed > 100) speed = 100;
            if (speed < 1) speed = 1;

            double delay = CalculateDelay(pan, tilt, (int)speed);

            bool success = true;
            if (servo == null) return false;

            this.status.IsMoving = true;
			this.headManager.OnHeadStatusChanged(this.status);
            
			success &= servo[0].SetSpeedPerc(speed);
            Thread.Sleep(2);
            success &= servo[1].SetSpeedPerc(speed);
            Thread.Sleep(2);

            if (!success)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Head: Can't set speeds");
            }
            success = true;

            TextBoxStreamWriter.DefaultLog.WriteLine("Head: Moving to " + pan + " " + tilt);
            success &= servo[0].SetPosition(pan);
            Thread.Sleep(20);
            success &= servo[1].SetPosition(tilt);

            if (!success)
            {
                System.Console.WriteLine("Head: Can't set angles: " + pan.ToString("0.00") + " " + tilt.ToString("0.00"));
                return false;
            }

            Thread.Sleep((int)(delay * 1000.0));

            this.status.GlobalPan = pan;
            this.status.GlobalTilt = tilt;

            headManager.UpdatePositionSharedVariable();

            this.status.IsMoving = false;
			this.headManager.OnHeadStatusChanged(this.status);

			return true;
        }
        
        public double CalculateDelay(double pan, double tilt, int speed_perc)
        {

            List<double> nums = new List<double>();
            double max, kDelay;
            double delay;

            nums.Add(Math.Abs(pan - this.status.GlobalPan));
            nums.Add(Math.Abs(tilt - this.status.GlobalTilt));

            max = nums.Max();

            kDelay =  9.0 / speed_perc;
            delay = kDelay * max;

            return delay;

        }
        
        public bool GetLookAtPosition(out double pan, out double tilt)
        {
            pan = this.status.GlobalPan;
            tilt = this.status.GlobalTilt;

            return true;
        }

        public bool stopTask()
        {
            
            bool success = true;
            
            success &= servo[0].SetSpeed(0);
            Thread.Sleep(20);
            
            success &= servo[0].SetPosition(this.status.GlobalPan);
            Thread.Sleep(20);

            success &= servo[1].SetSpeed(0);
            Thread.Sleep(20);
            
            success &= servo[1].SetPosition(this.status.GlobalTilt);
            Thread.Sleep(20);

            this.status.IsMoving = !success;

            return success;
        }
        #endregion


    }
}
    
