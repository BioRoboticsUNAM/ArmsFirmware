using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Xml.Serialization;
using Robotics.Controls;

namespace Head
{
    public class Settings
    {
        public int port;
        public string com;
        public int baud;
        public int idtilt;
        public int idpan;
        public ServoType servoTilt;
        public ServoType servoPan;
        public int zeroTilt;
        public int zeroPan;
        public double maxSpeedPerc;

        //Default settings constructor
        public Settings()
        {
            //Blackboard port
            this.port = 2090;
            //SerialPort
            this.com = "COM8";
            this.baud = 250000;
            //Servos
            this.idpan = 5;
            this.idtilt = 1;
            this.servoPan = ServoType.RX64;
            this.servoTilt = ServoType.RX64;
            this.zeroPan = 98;
            this.zeroTilt = 90;
            this.maxSpeedPerc = 20;
        }
    }
}
