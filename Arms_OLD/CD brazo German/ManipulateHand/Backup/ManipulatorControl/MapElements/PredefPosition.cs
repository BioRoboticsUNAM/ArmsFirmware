using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml.Serialization;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.ComponentModel;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public class PredefPosition : Drawable
    {
        private const int ellipseWidht = 10;    //Only even integers
        private const int ellipseHeight = 10;   //Only even integers
        private double time;
        private double gripper;
        protected Vector q;
        protected Vector x;
        private Manipulator arm;

        public PredefPosition(string name, Manipulator arm)
        {
            this.Name = name;
            if (position != null) this.Position = position;
            else this.Position = Vector3.Zero;
            this.Width = 0.1;
            this.Depth = 0.1;
            this.Height = 0.1;
            this.q = new Vector(7);
            this.x = new Vector(7);
            this.arm = arm;
        }

        public PredefPosition() : this("predefPos0", new Manipulator( new SerialPort(),ArmType.RightArm)) { }

        public static bool SerializeToXml(PredefPosition[] positions, string path)
        {
            XmlSerializer serializer = new XmlSerializer(typeof(PredefPosition[]));
            if (File.Exists(path))
            {
                string[] emptyStrings = { "" };
                File.WriteAllLines(path, emptyStrings);
            }
            else File.Create(path);

            try
            {
                Stream stream = File.OpenWrite(path);
                serializer.Serialize(stream, positions);
                stream.Close();
            }
            catch { return false; }
            return true;
        }

        public static PredefPosition[] DeserializeFromXml(string path)
        {
            try
            {
                XmlSerializer serializer = new XmlSerializer(typeof(PredefPosition[]));
                PredefPosition[] positions;
                Stream stream = File.OpenRead(path);
                positions = (PredefPosition[])serializer.Deserialize(stream);
                stream.Close();
                return positions;
            }
            catch { return null; }
        }

        #region Cartesian Coordinates

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the X-coordinate of the final effector")]
        public override double X
        {
            get
            {
                return this.x[0];
            }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[0] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                    this.Position.X = value;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid X value. Out of work space");
            }
        }

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the Y-coordinate of the final effector")]
        public override double Y
        {
            get
            {
                return this.x[1];
            }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[1] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                    this.Position.Y = value;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid Y value. Out of work space");
            }
        }

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the Z-coordinate of the final effector")]
        public override double Z
        {
            get
            {
                return this.x[2];
            }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[2] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                    this.Position.Z = value;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid Z value. Out of work space");
            }
        }

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the Roll orientation of the final effector")]
        public double Roll
        {
            get { return this.x[3]; }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[3] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid Roll value. Out of work space");
            }
        }

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the Pitch orientation of the final effector")]
        public double Pitch
        {
            get { return this.x[4]; }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[4] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid Pitch value. Out of work space");
            }
        }

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the Yaw orientation of the final effector")]
        public double Yaw
        {
            get { return this.x[5]; }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[5] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid Yaw value. Out of work space");
            }
        }

        [CategoryAttribute("Cartesian Coords"),
        DescriptionAttribute("Gets or sets the elbow orientation")]
        public double Elbow
        {
            get { return this.x[6]; }
            set
            {
                Vector tempX = new Vector(this.x);
                tempX[6] = value;
                Vector tempQ;
                if (this.arm.InverseKinematics(tempX, out tempQ))
                {
                    this.q = tempQ;
                    this.x = tempX;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid Elbow value. Out of work space");
            }
        }

        #endregion

        #region Articular Coordinates

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 1 in [rad]")]
        public double Q1
        {
            get { return this.q[0]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[0] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 2 in [rad]")]
        public double Q2
        {
            get { return this.q[1]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[1] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 3 in [rad]")]
        public double Q3
        {
            get { return this.q[2]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[2] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 4 in [rad]")]
        public double Q4
        {
            get { return this.q[3]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[3] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 5 in [rad]")]
        public double Q5
        {
            get { return this.q[4]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[4] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 6 in [rad]")]
        public double Q6
        {
            get { return this.q[5]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[5] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        [CategoryAttribute("Articular Coords"),
        DescriptionAttribute("Gets or sets the angle of the joint 7 in [rad]")]
        public double Q7
        {
            get { return this.q[6]; }
            set
            {
                double temp = value;
                while (temp > Math.PI) temp -= 2 * Math.PI;
                while (temp < -Math.PI) temp += 2 * Math.PI;
                this.q[6] = value;
                this.x = this.arm.ForwardKinematics(this.q);
            }
        }

        #endregion

        public double Gripper
        {
            get { return this.gripper; }
            set
            {
                if (value >= 0 && value <= 1) this.gripper = value;
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid gripper value");
            }
        }

        public double Time
        {
            get { return this.time; }
            set
            {
                if (value >= 0) this.time = value;
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid time value");
            }
        }

        #region Drawable Members

        public override int Layer
        {
            get { return 4; }
        }

        public static bool IsVisible { get; set; }

        public override bool Visible
        {
            get { return PredefPosition.IsVisible; }
        }

        public override void Draw(Graphics g)
        {
            Rectangle rect = new Rectangle();
            rect.X = (int)(this.X * Drawable.DrawingScaleW) - ellipseWidht / 2;
            rect.Y = (int)(Drawable.PlotterDepth - this.Y * Drawable.DrawingScaleD) - ellipseHeight / 2;
            rect.Width = ellipseWidht;
            rect.Height = ellipseHeight;
            g.FillEllipse(Brushes.Orange, rect);
        }

        public override bool ContainsPoint(Point Point)
        {
            return (Math.Abs(Point.X - (int)(this.X * Drawable.DrawingScaleW)) < ellipseWidht &&
                    Math.Abs(Drawable.PlotterDepth - Point.Y - (int)(this.Y * Drawable.DrawingScaleD)) < ellipseHeight);
        }

        public override bool ContainsRegion(Rectangle rect)
        {
            return false;
        }

        #endregion

        public override string ToString()
        {
            return this.Name;
        }
    }
}
