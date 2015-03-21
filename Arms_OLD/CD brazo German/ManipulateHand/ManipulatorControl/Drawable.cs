using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;
using System.Drawing;
using System.Xml.Serialization;
using Robotics.Controls;
using Robotics.Mathematics;

namespace ManipulatorControl
{
	public abstract class Drawable : IComparable<Drawable>
	{
		private static double drawingScaleW;
		private static double drawingScaleD;
		private static int plotterWidth;
		private static int plotterDepth;

		private string name;
		protected Vector3 position;
		private double width;
		private double depth;
		private double height;

		private Rectangle drawingRect; //Rectangle in pixels

		#region Static Member
		protected static double DrawingScaleW
		{
			get { return drawingScaleW; }
			set { if (value > 0) drawingScaleW = value; }
		}

		protected static double DrawingScaleD
		{
			get { return drawingScaleD; }
			set { if (value > 0) drawingScaleD = value; }
		}

		protected static int PlotterWidth
		{
			get { return plotterWidth; }
			set { if (value > 0) plotterWidth = value; }
		}

		protected static int PlotterDepth
		{
			get { return plotterDepth; }
			set { if (value > 0) plotterDepth = value; }
		}
		#endregion

		[Browsable(false)]
		public abstract int Layer { get; }

		[Browsable(false)]
		public abstract bool Visible { get; }

		public virtual void Draw(System.Drawing.Graphics g)
		{
			this.drawingRect.X = (int)((this.X - this.Width / 2) * Drawable.DrawingScaleW);
			this.drawingRect.Y = (int)(Drawable.PlotterDepth - (this.Y + this.Depth / 2) * Drawable.DrawingScaleD);
			this.drawingRect.Width = (int)(this.Width * Drawable.DrawingScaleW);
			this.drawingRect.Height = (int)(this.Depth * Drawable.DrawingScaleD);
		}

		/// <summary>
		/// Checks if the point "Point" is contained in the Rectangle given by "Position, Width and Depth" of
		/// this object. All units in [PIXELS] w.r.t MapPlotter
		/// </summary>
		/// <param name="Point">Point to be evaluated</param>
		/// <returns></returns>
		public virtual bool ContainsPoint(System.Drawing.Point Point)
		{
			return (Math.Abs(Point.X - (int)(this.X * Drawable.drawingScaleW)) < (int)(this.width / 2 * Drawable.drawingScaleW) &&
				Math.Abs(Drawable.PlotterDepth - Point.Y - (int)(this.Y * Drawable.drawingScaleD)) < (int)(this.depth / 2 * Drawable.drawingScaleD));
		}

		/// <summary>
		/// Checks if the rectangle "rect" is contained in the Rectangle given by "Position, Width and Depth" of
		/// this object. All units in [PIXELS] w.r.t MapPlotter
		/// </summary>
		/// <param name="rect">Rectangle to be evaluated</param>
		/// <returns></returns>
		public virtual bool ContainsRegion(System.Drawing.Rectangle rect)
		{
			return ((Math.Abs((rect.X + rect.Width / 2) - (int)(this.X * Drawable.drawingScaleW)) + rect.Width / 2) < (int)(this.width * Drawable.drawingScaleW / 2) &&
				   (Math.Abs((Drawable.PlotterDepth - rect.Y - rect.Height / 2) - this.Y * Drawable.drawingScaleD) + rect.Height / 2) < (int)(this.depth * Drawable.drawingScaleD / 2));
		}

		/// <summary>
		/// Calculates if the point "point" is contained in the cube given by "Position, Width, Depth and Height" 
		/// of this object. All units in [METERS]
		/// </summary>
		/// <param name="point">Point to be evaluated</param>
		/// <returns></returns>
		public virtual bool ContainsPoint(Vector3 point)
		{
			return ((Math.Abs(point.X - this.X) < this.width / 2) &&
					(Math.Abs(point.Y - this.Y) < this.depth / 2) &&
					(Math.Abs(point.Z - this.Z) < this.height / 2));
		}

		/// <summary>
		/// Calculates if the cube given by "centroid, width, depth and height" is contained in the cube given by
		/// "Position, Width, Depth and Height" of this object. All units in [METERS]
		/// </summary>
		/// <param name="centroid">Position of the cube to be evaluated w.r.t map frame</param>
		/// <param name="width">Widht of the cube to be evaluated [METERS]</param>
		/// <param name="depth">Depth of the cube to be evaluated [METERS]</param>
		/// <param name="heigth">Height of the cube to be evaluated [METERS]</param>
		/// <returns></returns>
		public virtual bool ContainsRegion(Vector3 centroid, double width, double depth, double heigth)
		{
			if (width < 0 || depth < 0 || height < 0) return false;
			return (((Math.Abs(centroid.X - this.X) + width / 2) < this.width / 2) &&
					((Math.Abs(centroid.Y - this.Y) + depth / 2) < this.depth / 2) &&
					((Math.Abs(centroid.Z - this.Z) + height / 2) < this.height / 2));
		}

		protected Rectangle DrawingRect { get { return this.drawingRect; } }

		/// <summary>
		/// Gets or sets the current object name
		/// </summary>
		[CategoryAttribute("(Name)"),
		DescriptionAttribute("Gets or sets the current object name")]
		public virtual string Name
		{
			get { return this.name; }
			set
			{
				if (value != null && value != "")
					this.name = value;
				else TextBoxStreamWriter.DefaultLog.WriteLine("Drawable: Invalid name");
			}
		}

		/// <summary>
		/// Gets or sets the object position w.r.t. the map frame [meters]
		/// </summary>
		[BrowsableAttribute(false)]
		[XmlIgnore]
		public virtual Vector3 Position
		{
			get { return this.position; }
			set
			{
				if (value != null) this.position = value;
				else TextBoxStreamWriter.DefaultLog.WriteLine("Drawable " + this.name + ": Invalid Position value");
			}
		}

		/// <summary>
		/// Gets or sets the centroid X coordinate w.r.t. map frame [meters]
		/// </summary>
		[CategoryAttribute("Position"),
		DescriptionAttribute("Gets or sets the centroid X coordinate w.r.t. map frame [meters]")]
		public virtual double X
		{
			get { return this.position.X; }
			set { this.position.X = value; }
		}

		/// <summary>
		/// Gets or sets the centroid Y coordinate w.r.t. map frame [meters]
		/// </summary>
		[CategoryAttribute("Position"),
		DescriptionAttribute("Gets or sets the centroid Y coordinate w.r.t. map frame [meters]")]
		public virtual double Y
		{
			get { return this.position.Y; }
			set { this.position.Y = value; }
		}

		/// <summary>
		/// Gets or sets the centroid Z coordinate w.r.t. map frame [meters]
		/// </summary>
		[CategoryAttribute("Position"),
		DescriptionAttribute("Gets or sets the centroid Z coordinate w.r.t. map frame [meters]")]
		public virtual double Z
		{
			get { return this.position.Z; }
			set { this.position.Z = value; }
		}

		/// <summary>
		/// Gets or sets the Width dimension of this object [meters]
		/// </summary>
		[CategoryAttribute("Size"),
		DescriptionAttribute("Gets or sets the Width dimension of this object [meters]")]
		public virtual double Width
		{
			get { return this.width; }
			set
			{
				if (value > 0) this.width = value;
				else TextBoxStreamWriter.DefaultLog.WriteLine("Drawable " + this.name + ": Width value must be greater than zero");
			}
		}

		/// <summary>
		/// Gets or sets the Depth dimension of this object [meters]
		/// </summary>
		[CategoryAttribute("Size"),
		DescriptionAttribute("Gets or sets the Depth dimension of this object [meters]")]
		public virtual double Depth
		{
			get { return this.depth; }
			set
			{
				if (value > 0) this.depth = value;
				else TextBoxStreamWriter.DefaultLog.WriteLine("Drawable " + this.name + ": Depth value must be greater than zero");
			}
		}

		/// <summary>
		/// Gets or sets the Height dimension of this object [meters]
		/// </summary>
		[CategoryAttribute("Size"),
		DescriptionAttribute("Gets or sets the Height dimension of this object [meters]")]
		public virtual double Height
		{
			get { return this.height; }
			set
			{
				if (value > 0) this.height = value;
				else TextBoxStreamWriter.DefaultLog.WriteLine("Drawable " + this.name + ": Height value must be greater than zero");
			}
		}

		#region IComparable<Drawable> Members

		public int CompareTo(Drawable other)
		{
			return this.Layer - other.Layer;
		}

		#endregion
	}
}
