using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml.Serialization;
using System.Drawing;
using System.IO;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
    [Serializable]
    public class PredefMovement : Drawable
    {
        private List<PredefPosition> positions;

        public PredefMovement(string name)
        {
            this.Name = name;
            this.position = Vector3.Zero;
            this.positions = new List<PredefPosition>();
        }

        public PredefMovement() : this("predefMov0") { }

        public static bool SerializeToXml(PredefMovement[] movs, string path)
        {
            XmlSerializer serializer = new XmlSerializer(typeof(PredefMovement[]));
            if (File.Exists(path))
            {
                string[] emptyStrings = { "" };
                File.WriteAllLines(path, emptyStrings);
            }
            else
                File.Create(path);


            try
            {
                Stream stream = File.OpenWrite(path);
                serializer.Serialize(stream, movs);
                stream.Close();
            }
            catch
            {
                return false;
            }
            return true;
        }

        public static PredefMovement[] DeserializeFromXml(string path)
        {
            try
            {
                XmlSerializer serializer = new XmlSerializer(typeof(PredefMovement[]));
                PredefMovement[] movs;
                Stream stream = File.OpenRead(path);
                movs = (PredefMovement[])serializer.Deserialize(stream);
                stream.Close();
                return movs;
            }
            catch
            {
                return null;
            }
        }

        #region Drawable Members

        public override int Layer
        {
            get { return 3; }
        }

        public static bool IsVisible { get; set; }

        public override bool Visible
        {
            get { return PredefMovement.IsVisible; }
        }

        public override void Draw(System.Drawing.Graphics g)
        {
            return;
            /*
            if (this.optimalPath == null || this.optimalPath.Count < 2) return;

            this.pointsToDraw.Clear();
            foreach (Vector3 v in this.optimalPath)
                this.pointsToDraw.Add(new Point((int)(v.X * Drawable.DrawingScaleW),
                    (int)(Drawable.PlotterDepth - v.Y * Drawable.DrawingScaleD)));
            g.DrawLines(Pens.Blue, this.pointsToDraw.ToArray());
            */
        }

        public override bool ContainsPoint(Robotics.Mathematics.Vector3 point)
        {
            return false;
        }

        public override bool ContainsPoint(System.Drawing.Point Point)
        {
            return false;
        }

        public override bool ContainsRegion(Robotics.Mathematics.Vector3 centroid, double width, double depth, double heigth)
        {
            return false;
        }

        public override bool ContainsRegion(System.Drawing.Rectangle rect)
        {
            return false;
        }

        #endregion

        public List<PredefPosition> Positions
        {
            get { return this.positions; }
            set
            {
                if (value != null) this.positions = value;
                else TextBoxStreamWriter.DefaultLog.WriteLine(this.Name + ": Invalid list of predefined positions");
            }
        }

        public override string ToString()
        {
            return this.Name;
        }
    }
}
