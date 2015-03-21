using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.IO.Ports;
using System.Threading;
using Robotics.Controls;

namespace Head
{
	public enum ServoType { AX12, RX64, EX106, RX28 }

	public class ServoControl
	{
		
		#region VARIABLES

		public readonly ServoType dynamixelType;		// Dynamixel Type "RX-64", "AX-12"
		protected readonly int id;						// Servo ID
		protected readonly double[] mechanicalLimits;	// 
		protected double ceroPosition;					// Absolute position of the mathematic model cero in the servo range
		protected readonly bool cw;						// Clockwise angle measurement
		protected readonly double maxDegrees;				// Maximum servorange in degrees
		protected readonly int maxBits;					// Maximum servo range in register bits.
		protected readonly double negativeRange;		// Negative range from 0 degrees 
		protected readonly double[] softwareLimits;		//
		protected bool dualServo;

		protected readonly SerialPort serialPort;		// Manipulator SerialPort used in ServoControl
		protected SerialPortManager serialPortManager;
				
		#region Status

		protected bool[] errors;				// Bool array of servo errors
		protected bool pingOK;					// Logical 'OR' for all servo errors
		protected bool torqueEnabled;			// Servo Torque Enabled True/False
		protected double torqueLimitPerc;		// Maximum Torque Limit in Percentage
		protected int servoStatus;				// Servo Status 0 - Error State; 1 - No error state && ( Torque Enabled == False || Torque Limit Percentaje < 80%); 2 - No error state && Torque Enabled: True && Torque Limit Percentaje > 80%
		protected double temp;					// Servo Temp
		protected double load;					// Servo Load
		protected double voltage;	 			// Supply Voltage
		protected double maxServoSpeed;			// Maximum Sevo Speed. Function of Supply Voltage and DataSheet calculated Constant
		protected double maxServoTorque;		// Maximum Servo Torque. Function of Supply Volgate and DataSheet calculated Constant
		protected bool reportExecuted;
		#endregion


		#endregion

		#region CONSTRUCTOR
		
		/// <summary>
		/// Creates Servo Object
		/// </summary>
		/// <param name="dynamixelType">Determines Dynamixel Type for speed and torque calculation</param>
		/// <param name="id">Dynamixel ID</param>
		/// <param name="ceroPosition">Place of the cero reference. Use normal reference  180° <-- --> 0°</param>
		/// <param name="cw">ClockWise motion</param>
		/// <param name="port">Serial Port to use</param>
		public ServoControl(ServoType dynamixelType, int id, double ceroPosition, bool cw, SerialPortManager serialPortManager)
		{
			this.dynamixelType = dynamixelType;
			this.id = id;
			this.mechanicalLimits = new double[2];
			this.softwareLimits = new double[2];
			this.CeroPosition = ceroPosition;
			this.cw = cw;
			this.serialPortManager = serialPortManager;
			this.dualServo = false;
			
			// Status

			errors = new bool[7];
            for (int i = 0; i < errors.Length; i++) { errors[i] = true; }
			pingOK = false;
			torqueEnabled = false;
			torqueLimitPerc = 0;
			servoStatus = 0;
			temp = 0;
			load = 0;
			reportExecuted = true;

			// Servo Speed and Torque

			switch (dynamixelType)
			{
				case ServoType.RX64:

					voltage = 18;
					this.maxServoSpeed = (voltage * 0.366616119) + 0.070957958;	  // From DataSheet
					this.maxServoTorque = (voltage * 4.266666667) + 0.4;			  // From Datasheet
					this.maxBits = 1023;
					this.maxDegrees = 300;
					this.negativeRange = (this.maxDegrees - 180.0) / 2.0;
					break;
				case ServoType.AX12:
					voltage = 10;
					this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
					this.maxServoTorque = (voltage * 1.5) + 1.5;			  // From Datasheet
					this.maxBits = 1023;
					this.maxDegrees = 300;
					this.negativeRange = (this.maxDegrees - 180.0) / 2.0;
					break;

				case ServoType.EX106:
					voltage = 18;
					this.maxServoSpeed = (voltage * 0.424115432) - 0.607898786;	  // From DataSheet
					this.maxServoTorque = (voltage * 5.945945946) - 4;			  // From Datasheet
					this.maxBits = 4095;
					//this.maxDegrees = 280.6;
					this.maxDegrees = 250.92;
					this.negativeRange = (this.maxDegrees - 180.0) / 2.0;
					break;
				case ServoType.RX28:
					voltage = 18;
					this.maxServoSpeed = (voltage * 0.3400778487) + 2.189709835;	  // From DataSheet
					this.maxServoTorque = (voltage * 1.566666666667) + 9.5;			  // From Datasheet
					this.maxBits = 1023;
					//this.maxDegrees = 280.6;
					this.maxDegrees = 300;
					this.negativeRange = (this.maxDegrees - 180.0) / 2.0;
					break;

				default:
					voltage = 10;
					this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
					this.maxServoTorque = (voltage * 1.5) + 1.5;			  // From Datasheet
					this.maxBits = 1023;
					this.maxDegrees = 300;
					this.negativeRange = (this.maxDegrees - 180.0) / 2.0;
					break;
			}

			//manipulatorStruct.serialPortManager.CommandReceived += new StringEventHandler(serialPortManager_CommandReceived);

		}
		
		
		

		#endregion

		#region PROPERTIES
		/// <summary>
		/// Servo ID
		/// </summary>
		public int Id {
			get {
				return id;
			}
		}
		/// <summary>
		/// Place of cero position 180° <-- --> 0°
		/// </summary>
		public double CeroPosition {
			get {
				return ceroPosition;
			}
			set
			{
			 ceroPosition = value;
			 mechanicalLimits[0] = -(this.ceroPosition + 60);
			 mechanicalLimits[1] = 240 - this.ceroPosition;
			 
			}
		}
		/// <summary>
		/// ClockWise motion
		/// </summary>
		public bool CW {
			get {
				return cw;
			}
			
		
		}
		/// <summary>
		/// Voltage
		/// </summary>
		public virtual double Voltage {
			get {
				return voltage;
			}
			set {
				voltage = value;
				switch (dynamixelType)
				{
					case ServoType.RX64:
					  
						this.maxServoSpeed = (voltage * 0.366616119) + 0.070957958;	  // From DataSheet
						this.maxServoTorque = (voltage * 4.266666667) + 0.4;			  // From Datasheet
						break;
					case ServoType.AX12:
					  
						this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
						this.maxServoTorque = (voltage * 1.5) + 1.5;			  // From Datasheet
						break;
					default:
					  
						this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
						this.maxServoTorque = (voltage * 1.5) + 1.5;			  // From Datasheet
						break;

				}
			}
		}
		/// <summary>
		/// Load
		/// </summary>
		public virtual double Load {

			get { return load; }
			set { load = value; }
		}
		/// <summary>
		/// Temp
		/// </summary>
		public virtual double Temp {

			get { return temp; }
			set {
				temp = value;
			}
		}
		/// <summary>
		/// Maximum Speed of the Servo
		/// </summary>
		public double MaxServoSpeed
		{
			get
			{
				maxServoSpeed = (voltage * 0.366616119) + 0.070957958;	  // From DataSheet
				return maxServoSpeed;
			}
		}
		/// <summary>
		/// Maximum Torque of the Servo
		/// </summary>
		public double MaxServoTorque {
			get {
				maxServoTorque = (voltage * 4.266666667) + 0.4;			 // From Datasheet
				return maxServoTorque;
			}
		}
		/// <summary>
		/// Dual Servo
		/// </summary>
		public bool DualServo {
			get {
				return dualServo;
			}
		}

		//Status

		public virtual bool PingOK {

			get {
				
				pingOK = false;
				for(int i = 0;i<errors.Length;i++){
					pingOK = pingOK || errors[i]; ;
				}
				return !pingOK;
			

			}
		
		
		}
		public virtual bool[] Errors
		{
			get {
				//Ping();
				return errors;
			
			}
		
		}
		public virtual bool TorqueEnabled {
			get {
				return torqueEnabled;
			}
		}
		public virtual double TorqueLimitPerc {
			get {
				return torqueLimitPerc;
			}
		}
		public virtual int ServoStatus
		{
			get
			{
				//if (!GlobalStatus()) return 0;

				if (!PingOK) {
					this.servoStatus = 0;
					return servoStatus;
				}
				if ((torqueEnabled) && (torqueLimitPerc > 80))
				{
					this.servoStatus = 2;
					return servoStatus;
				}
				else {
					this.servoStatus = 1;
					return servoStatus;
				}

				

			}

		}
		public virtual bool IsReady{

			get {

				//if (!GlobalStatus()) return false;

				if (this.ServoStatus == 2)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
		}
		public virtual bool IsAlive
		{

			get
			{

				//if (!GlobalStatus()) return false;

				if (this.ServoStatus == 1)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
		}

		#endregion

		#region METHODS

		#region Position

		/// <summary>
		/// Sets servo position in bites. Instantaneously executed.
		/// </summary>
		/// <param name="thetaBits">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetPositionBits(int thetaBits)
		{

			byte[] Cmd;
			int[] thetaBitsParts = new int[2];

			if (thetaBits <= this.maxBits && thetaBits >= 0)
			{
				if (this.cw) thetaBits = this.maxBits - thetaBits;
				thetaBitsParts[0] = thetaBits % 256;
				thetaBitsParts[1] = thetaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of Position Range");
				return false;
			}


			Cmd = WriteServo(this.id, 30, 2, thetaBitsParts);
			return SendCommand(Cmd, "Set Position");
			
			

		} //OVERRIDE
		/// <summary>
		/// Sets position of the desired servo in bites. Instantaneously executed.
		/// </summary>
		/// <param name="id">Dynamixel ID</param>
		/// <param name="thetaBits">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetPositionBits(int id,int thetaBits)
		{

			byte[] Cmd;
			int[] thetaBitsParts = new int[2];

			if (thetaBits <= this.maxBits && thetaBits >= 0)
			{
				if (this.cw) thetaBits = this.maxBits - thetaBits;
				thetaBitsParts[0] = thetaBits % 256;
				thetaBitsParts[1] = thetaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of Position Range");
				return false;
			}


			Cmd = WriteServo(id, 30, 2, thetaBitsParts);
			return SendCommand(Cmd, "Set Position");
			

		} //OVERRIDE
		/// <summary>
		///  Sets servo position. Instantaneously executed.
		/// </summary>
		/// <param name="theta">double Angle [radians]</param>
		/// <returns>bool</returns>
		public virtual bool SetPosition(double theta) {

			int thetaBits;

			if (this.cw)
			{
				//theta = (-(ToRadians(90 - this.ceroPosition) + theta));
				theta = (this.maxDegrees - (this.ceroPosition + this.negativeRange)) + ToDegrees(theta);
				thetaBits = Convert.ToInt16(Math.Round((theta)) * (this.maxBits / (this.maxDegrees)));
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(90 + 60)) * (1023 / ToRadians(300))));
			}
			else {
				thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
			}

			
			return SetPositionBits(thetaBits);

		}//OVERRIDE
		/// <summary>
		///  Sets position of the desired Servo. Instantaneously executed.
		/// </summary>
		/// <param name="theta">double Angle [radians]</param>
		/// <returns>bool</returns>
		public bool SetPosition(int id, double theta)
		{

			int thetaBits;

			if (this.cw)
			{
				//theta = theta - ToRadians(this.ceroPosition - 180);
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + 60)) * (1023 / ToRadians(300))));
				theta = (this.maxDegrees - (this.ceroPosition + this.negativeRange)) + ToDegrees(theta);
				thetaBits = Convert.ToInt16(Math.Round((theta)) * (this.maxBits / (this.maxDegrees)));
			}
			else
			{
				thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
			}
			
			return SetPositionBits(id, thetaBits);

		}  
		/// <summary>
		/// Sets buffer servo position in bites. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetPositionBitsWait(int thetaBits)
		{

			byte[] Cmd;
			int[] thetaBitsParts = new int[2];



			if (thetaBits <= this.maxBits && thetaBits >= 0)
			{
				if (this.cw) thetaBits = this.maxBits - thetaBits;
				thetaBitsParts[0] = thetaBits % 256;
				thetaBitsParts[1] = thetaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of Position Range");
				return false;
			}


			Cmd = RegWriteServo(this.id, 30, 2, thetaBitsParts);
			return SendCommand(Cmd, "Set Position Wait");
			

		}  //OVERRIDE
		/// <summary>
		/// Sets buffer position of the desired Servo in bites. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool SetPositionBitsWait(int id,int thetaBits)
		{

			byte[] Cmd;
			int[] thetaBitsParts = new int[2];



			if (thetaBits <= this.maxBits && thetaBits >= 0)
			{
				if (this.cw) thetaBits = this.maxBits - thetaBits;
				thetaBitsParts[0] = thetaBits % 256;
				thetaBitsParts[1] = thetaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of Position Range");
				return false;
			}


			Cmd = RegWriteServo(id, 30, 2, thetaBitsParts);

			return SendCommand(Cmd, "Set Position Wait");
		}  
		/// <summary>
		/// Sets buffer servo position. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="theta">double Angle [radians]</param>
		/// <returns>bool</returns>
		public virtual bool SetPositionWait(double theta)
		{
			
			int thetaBits;
			if (this.cw)
			{
				//theta = theta - ToRadians(this.ceroPosition - 180);
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
				theta = (this.maxDegrees - (this.ceroPosition + this.negativeRange)) + ToDegrees(theta);
				thetaBits = Convert.ToInt16(Math.Round((theta)) * (this.maxBits / (this.maxDegrees)));
			}
			else
			{
				thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
			}
			
			return SetPositionBitsWait(thetaBits);
		}  //OVERRIDE
		/// <summary>
		/// Sets buffer position of the desired Servo. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="theta">double Angle [radians]</param>
		/// <returns>bool</returns>
		public bool SetPositionWait(int id, double theta)
		{

			int thetaBits;
			if (this.cw)
			{
				//theta = theta - ToRadians(this.ceroPosition - 180);
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
				theta = (this.maxDegrees - (this.ceroPosition + this.negativeRange)) + ToDegrees(theta);
				thetaBits = Convert.ToInt16(Math.Round((theta)) * (this.maxBits / (this.maxDegrees)));
			}
			else
			{
				thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
			}

			return SetPositionBitsWait(id, thetaBits);
		}
		
		/// <summary>
		/// Gets current servo position in Bits
		/// </summary>
		/// <param name="positionBits">out int position</param>
		/// <returns>bool</returns>
		public bool GetPositionBits(out int positionBits) {

			byte[] Cmd;
			byte[] inCmd;
			positionBits = 0;
			
			Cmd = ReadServo(this.id, 36, 2);
			
			//if (!SendCommand(Cmd, "Get Position"))
			//{				positionBits = 0;
			//	return false;
			//}
			//serialPort.ReceivedBytesThreshold = 8;
			//if (!ReceiveCommandAsync(out inCmd)) return false;
			if (!SendAndReceiveCommand(Cmd, "Get Position", out inCmd, 2)) return false;
			
			positionBits = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));
			if (this.cw) positionBits = this.maxBits - positionBits;
						
			return true;

		}
		/// <summary>
		/// Gets current position of the desired Servo
		/// </summary>
		/// <param name="positionBits">out int position</param>
		/// <returns>bool</returns>
		public bool GetPositionBits(int id, out int positionBits)
		{

			byte[] Cmd;
			byte[] inCmd = new byte[2];
			
			positionBits = 0;
				
			Cmd = ReadServo(id, 36, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Position", out inCmd, 2)) return false;

				positionBits = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));
				if (this.cw) positionBits = this.maxBits - positionBits;
			
			return true;

		}
		/// <summary>
		/// Gets current servo position
		/// </summary>
		/// <param name="position">out double position [rad]</param>
		/// <returns>bool</returns>
		public bool GetPosition(out double position)
		{
			int positionBits;

			if (!GetPositionBits(out positionBits))
			{
				position = 0;
				return false;
			}
			else
			{
				if (this.cw) {
					positionBits = this.maxBits - positionBits;
				}

				position = ToRadians(((positionBits * this.maxDegrees) / this.maxBits) - (this.ceroPosition + this.negativeRange));

				if (this.cw) {
				    position = ToRadians((this.ceroPosition + this.negativeRange) - ((positionBits * this.maxDegrees) / this.maxBits));
				}
				return true;

			}
		}
		/// <summary>
		/// Gets current position of the desired Servo
		/// </summary>
		/// <param name="position">out double position [rad]</param>
		/// <returns>bool</returns>
		public bool GetPosition(int id, out double position)
		{
			int positionBits;

			if (!GetPositionBits(id, out positionBits))
			{
				position = 0;
				return false;
			}
			else
			{


				position = ToRadians(((positionBits * this.maxDegrees) / this.maxBits) - (this.ceroPosition + this.negativeRange));

				if (this.cw)
				{
					position = ToRadians((this.ceroPosition + this.negativeRange) - ((positionBits * this.maxDegrees) / this.maxBits));
				}
				return true;

			}
		}
	   
		#endregion

		#region Speed
		/// <summary>
		/// Sets servo speed in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="omegaBits">int Speed [0,1023[bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeedBits(int omegaBits)
		{

			byte[] Cmd;
			int[] omegaBitsParts = new int[2];

			if (omegaBits <= 1023 && omegaBits > 0)
			{
				omegaBitsParts[0] = omegaBits % 256;
				omegaBitsParts[1] = omegaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of Speed Range");
				return false;
			}

			Cmd = WriteServo(this.id, 32, 2, omegaBitsParts);
			if (!SendCommand(Cmd, "Set Speed")) return false;
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Speed",this.serialPort)) return false;
			return true;

		} //OVERRIDE
		/// <summary>
		/// Sets speed of the desired Servo in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="omegaBits">int Speed [0,1023[bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeedBits(int id, int omegaBits)
		{

			byte[] Cmd;
			int[] omegaBitsParts = new int[2];

			if (omegaBits <= 1023 && omegaBits > 0)
			{
				omegaBitsParts[0] = omegaBits % 256;
				omegaBitsParts[1] = omegaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of Speed Range");
				return false;
			}

			Cmd = WriteServo(id, 32, 2, omegaBitsParts);
			if (!SendCommand(Cmd, "Set Speed")) return false;
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Speed",this.serialPort)) return false;
			return true;

		} //OVERRIDE
		/// <summary>
		/// Sets Sevro Speed.Instantaneously executed.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeedPerc(double omega)
		{

			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / 100.0, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBits(speed);

		}  //OVERRIDE
		/// <summary>
		/// Sets Sevro Speed.Instantaneously executed.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeed(double omega){

			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / this.maxServoSpeed, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBits(speed);

					}  //OVERRIDE


		/// <summary>
		/// Sets buffer servo speed in bites. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="omegaBits">double Speed [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeedBitsWait(int omegaBits)
		{

			byte[] Cmd;
			int[] omegaBitsParts = new int[2];

			if (omegaBits <= 1023 && omegaBits > 0)
			{
				omegaBitsParts[0] = omegaBits % 256;
				omegaBitsParts[1] = omegaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of Speed Range");
				return false;
			}

			Cmd = RegWriteServo(this.id, 32, 2, omegaBitsParts);
			if (!SendCommand(Cmd, "Set Speed Wait")) return false;
//			if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Position", this.serialPort)) return false;
			return true;

		} //OVERRIDE
		/// <summary>
		/// Sets buffer speed of the desired Servo in bites. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="omegaBits">double Speed [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool SetSpeedBitsWait(int id, int omegaBits)
		{

			byte[] Cmd;
			int[] omegaBitsParts = new int[2];

			if (omegaBits <= 1023 && omegaBits > 0)
			{
				omegaBitsParts[0] = omegaBits % 256;
				omegaBitsParts[1] = omegaBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of Speed Range");
				return false;
			}

			Cmd = RegWriteServo(id, 32, 2, omegaBitsParts);
			if (!SendCommand(Cmd, "Set Speed Wait")) return false;
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Position",this.serialPort)) return false;
			return true;

		}
		/// <summary>
		/// Sets buffer servo speed. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeedWaitPerc(double omega)// OVERRIDE
		{

			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / 100.0, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBitsWait(speed);

		}
		/// <summary>
		/// Sets buffer servo speed. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public virtual bool SetSpeedWait(double omega)// OVERRIDE
		{

			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / this.maxServoSpeed, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBitsWait(speed);

		}
	   
		/// <summary>
		/// Gets current servo speed in bits
		/// </summary>
		/// <param name="positionBits">out int speed in bits [0,1023]</param>
		/// <returns>bool</returns>
		public bool GetCurrentSpeedBits(out int speedBits)
		{

			byte[] Cmd;
			byte[] inCmd;
			speedBits = 0;
			

			Cmd = ReadServo(this.id, 38, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Current Speed", out inCmd, 2)) return false;

				speedBits = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));
				
			
			return true;

		}
		/// <summary>
		/// Gets current speed of the desired Servo
		/// </summary>
		/// <param name="positionBits">out int speed in Bits</param>
		/// <returns>bool</returns>
		public bool GetCurrentSpeedBits(int id, out int speedBits)
		{

			byte[] Cmd;
			byte[] inCmd;
			speedBits = 0;
			Cmd = ReadServo(id, 38, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Current Speed", out inCmd, 2)) return false;

				speedBits = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));

			
			return true;

		}
		/// <summary>
		/// Gets current servo speed
		/// </summary>
		/// <param name="position">Out double speed [rad/sec]</param>
		/// <returns>bool</returns>
		public bool GetCurrentSpeed(out double speed)
		{
			int speedBits;

			if (!GetCurrentSpeedBits(out speedBits))
			{
				speed = 0;
				return false;
			}
			else
			{
				speed = speedBits * (this.maxServoSpeed) / 1023;
				return true;
			}
		}
		/// <summary>
		/// Gets current speed of the desired Servo
		/// </summary>
		/// <param name="position">Out double speed [rad/sec]</param>
		/// <returns>bool</returns>
		public bool GetCurrentSpeed(int id, out double speed)
		{
			int speedBits;

			if (!GetCurrentSpeedBits(id, out speedBits))
			{
				speed = 0;
				return false;
			}
			else
			{
				speed = speedBits * (this.maxServoSpeed) / 1023;
				return true;
			}
		}

		/// <summary>
		/// Gets servo programmed speed
		/// </summary>
		/// <param name="positionBits">out int speed in Bits</param>
		/// <returns>bool</returns>
		public bool GetSpeedBits(out int speedBits)
		{

			byte[] Cmd;
			byte[] inCmd;
			speedBits = 0;


			Cmd = ReadServo(this.id, 32, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Speed", out inCmd, 2)) return false;

				speedBits = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));

			
			return true;

		}
		/// <summary>
		/// Gets programmed speed of the desired Servo.
		/// </summary>
		/// <param name="positionBits">out int speed in Bits</param>
		/// <returns>bool</returns>
		public bool GetSpeedBits(int id, out int speedBits)
		{

			byte[] Cmd;
			byte[] inCmd;
			speedBits = 0;


			Cmd = ReadServo(id, 32, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Speed", out inCmd, 2)) return false;

				speedBits = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));

			
			return true;

		}
		/// <summary>
		/// Gets servo programmed speed
		/// </summary>
		/// <param name="position">Out double speed [rad/sec]</param>
		/// <returns>bool</returns>
		public bool GetSpeed(out double speed)
		{
			int speedBits;

			if (!GetSpeedBits(out speedBits))
			{
				speed = 0;
				return false;
			}
			else
			{
				speed = speedBits * (this.maxServoSpeed) / 1023;
				return true;
			}
		}
		#endregion

		#region Torque

		/// <summary>
		/// Sets servo torque in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="direction">0 - CCW, 1 - CW</param>
		/// <param name="torqueBits">torque in bits [0,1023]</param>
		/// <returns>bool</returns>
		public virtual bool SetTorqueBits(int direction, int torqueBits)
		{

			byte[] Cmd;
			int[] torqueBitsParts = new int[2];

			if (torqueBits <= 1023 && torqueBits >= 0)
			{

			if (direction == 1) {
				torqueBits = torqueBits + 1024;
			}
			
				torqueBitsParts[0] = torqueBits % 256;
				torqueBitsParts[1] = torqueBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of Torque Range");
				return false;
			}

			Cmd = WriteServo(this.id, 32, 2, torqueBitsParts);
			if (!SendCommand(Cmd, "Set Torque")) return false;
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Speed",this.serialPort)) return false;
			return true;

		} //
		/// <summary>
		/// Sets servo torque in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="id">ID of the desired Servo</param>
		/// <param name="direction">0 - CCW, 1 - CW</param>
		/// <param name="torqueBits">torque in bits [0,1023]</param>
		/// <returns>bool</returns>
		public bool SetTorqueBits(int id, int direction, int torqueBits)
		{

			byte[] Cmd;
			int[] torqueBitsParts = new int[2];

			if (direction == 1)
			{
				torqueBits = torqueBits + 1024;
			}
			if (torqueBits <= 2047 && torqueBits >= 0)
			{
				torqueBitsParts[0] = torqueBits % 256;
				torqueBitsParts[1] = torqueBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of Torque Range");
				return false;
			}

			Cmd = WriteServo(id, 32, 2, torqueBitsParts);
			if (!SendCommand(Cmd, "Set Torque")) return false;
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Speed",this.serialPort)) return false;
			return true;

		} //
		/// <summary>
		/// Set Torque in percentage
		/// </summary>
		/// <param name="torquePercentaje">double Percentaje [0,100] %</param>
		/// <returns>bool</returns>
		public virtual bool SetTorquePercentaje(double torquePercentaje){
		int direction;
		int torqueBits;

		if(torquePercentaje <= 100 && torquePercentaje >=-100){
		
			if(this.cw){

				if(torquePercentaje >=0){
			direction = 1;
				torquePercentaje = Math.Abs(torquePercentaje);
			}else{
			direction = 0;
				torquePercentaje = Math.Abs(torquePercentaje);
			}

			}
			else{
			if(torquePercentaje >=0){
			direction = 0;
				torquePercentaje = Math.Abs(torquePercentaje);
			}else{
			direction = 1;
				torquePercentaje = Math.Abs(torquePercentaje);
			}
			}

			
		}else{
			direction = 0;
			return false;
		}
		
			torqueBits = Convert.ToInt16((torquePercentaje * 1023.0) / 100.0);
			return SetTorqueBits(direction,torqueBits);
		
		
		}
		/// <summary>
		/// Set torque in Kgf.cm
		/// </summary>
		/// <param name="torque">double Torque [0,maxServoTorque][kgf.cm]</param>
		/// <returns>bool</returns>
		public virtual bool SetTorque(double torque)
		{
			int direction;
			int torqueBits;

			
				if (this.cw)
				{

					if (torque >= 0)
					{
						direction = 1;
						torque = Math.Abs(torque);
					}
					else
					{
						direction = 0;
						torque = Math.Abs(torque);
					}

				}
				else
				{
					if (torque >= 0)
					{
						direction = 0;
						torque = Math.Abs(torque);
					}
					else
					{
						direction = 1;
						torque = Math.Abs(torque);
					}
				}


			

			torqueBits = Convert.ToInt16((torque * 1023.0) / this.MaxServoTorque);
			if (torqueBits > 1023) {
				torqueBits = 1023;
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id.ToString() + " Max Servo Torque Exceeded");
			}


			return SetTorqueBits(direction, torqueBits);


		}

		#endregion

		#region Servo Configuration
		/// <summary>
		/// Sets servo CW Limit in bites. Instantaneously executed.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetMaxCWBits(int bits)
		{
			byte[] Cmd;
			int[] bitsParts = new int[2];
			
			if (bits <= this.maxBits && bits >= 0)
			{
				bitsParts[0] = bits % 256;
				bitsParts[1] = bits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of CW Limit Range");
				return false;
			}


			Cmd = WriteServo(this.id, 6, 2, bitsParts);
			return SendCommand(Cmd, "Set CW Limit");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set CW Limit", this.serialPort)) return false;
			

		}  // OVERRIDE
		/// <summary>
		/// Sets CW Limit of the desired Servo in bites. Instantaneously executed.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool SetMaxCWBits(int id, int bits)
		{
			byte[] Cmd;
			int[] bitsParts = new int[2];
			
			if (bits <= this.maxBits && bits >= 0)
			{
				bitsParts[0] = bits % 256;
				bitsParts[1] = bits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of CW Limit Range");
				return false;
			}


			Cmd = WriteServo(id, 6, 2, bitsParts);
			return SendCommand(Cmd, "Set CW Limit");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set CW Limit", this.serialPort)) return false;
			
		}
		/// <summary>
		/// Sets servo CW Limit. Reference is Servo Cero Position.
		/// </summary>
		/// <param name="angle">Angle [degrees]</param>
		/// <returns>bool</returns>
		public virtual bool SetMaxCW(double angle)
		{
			int thetaBits;
			if (angle >= mechanicalLimits[0])
			{
				thetaBits = Convert.ToInt16(Math.Round((angle + (this.ceroPosition + this.negativeRange)) * (this.maxBits / (this.maxDegrees))));
				return SetMaxCWBits(thetaBits);
			}
			else {
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + ": CW Limit is less than Mechanical Limit");
				return false;
			}
		}  //OVERRID
		
		/// <summary>
		/// Sets servo CCW Limit in bites. Instantaneously executed.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetMaxCCWBits(int bits)
		{
			byte[] Cmd;
			int[] bitsParts = new int[2];
			
			if (bits <= this.maxBits && bits >= 0)
			{
				bitsParts[0] = bits % 256;
				bitsParts[1] = bits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + " is out of CCW Limit Range");
				return false;
			}


			Cmd = WriteServo(this.id, 8, 2, bitsParts);
			return SendCommand(Cmd, "Set CCW Limit");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set CCW Limit", this.serialPort)) return false;
			
		} // OVERRIDE
		/// <summary>
		/// Sets CCW Limit of the desired Servo in bites. Instantaneously executed.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool SetMaxCCWBits(int id, int bits)
		{
			byte[] Cmd;
			int[] bitsParts = new int[2];
			
			if (bits <= this.maxBits && bits >= 0)
			{
				bitsParts[0] = bits % 256;
				bitsParts[1] = bits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + " is out of CCW Limit Range");
				return false;
			}


			Cmd = WriteServo(id, 8, 2, bitsParts);
			return SendCommand(Cmd, "Set CCW Limit");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set CCW Limit", this.serialPort)) return false;
			
		} 
		/// <summary>
		/// Sets servo CCW Limit. Reference is Servo Cero Position.
		/// </summary>
		/// <param name="angle"></param>
		/// <returns></returns>
		public virtual bool SetMaxCCW(double angle) {
			
			int thetaBits;
			if (angle <= mechanicalLimits[1])
			{
				thetaBits = Convert.ToInt16(Math.Round((angle + (this.ceroPosition + this.negativeRange)) * (this.maxBits / (this.maxDegrees))));
				return SetMaxCCWBits(thetaBits);
			}
			else {
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + ": CCW Limit is greater than Mechanical Limit");
				return false;
			}
		
		}  //OVERRIDE
		
		/// <summary>
		/// Enables Torque Servo Control
		/// </summary>
		/// <param name="enable">bool enable</param>
		/// <returns>bool</returns>
		public virtual bool SetTorqueControl(bool enable) {

			if (enable)
			{
				if (!SetMaxCWBits(0)) return false;
				if (!SetMaxCCWBits(0)) return false;
			}
			else {
				if (!SetMaxCWBits(1)) return false;
				if (!SetMaxCCWBits(this.maxBits)) return false;
			}

			return true;

		}
		/// <summary>
		/// Enables Torque Servo Control of the desired Servo
		/// </summary>
		/// <param name="id"></param>
		/// <param name="enable"></param>
		/// <returns></returns>
		public bool SetTorqueControl(int id, bool enable)
		{

			if (enable)
			{
				if (!SetMaxCWBits(id, 0)) return false;
				if (!SetMaxCCWBits(id, 0)) return false;
			}
			else
			{
				if (!SetMaxCWBits(id,1)) return false;
				if (!SetMaxCCWBits(id, this.maxBits)) return false;
			}

			return true;

		}
	   
		/// <summary>
		/// Enables Servo Torque
		/// </summary>
		/// <param name="enable">0 - Disable Torque. 1 - Enable Torque</param>
		/// <returns>Bool.</returns>
		public virtual bool TorqueEnable(int enable)
		{
			byte[] Cmd;
			if (enable == 0 || enable == 1)
			{
				Cmd = WriteServo(this.id, 24, 1, enable);
				return SendCommand(Cmd, "Set Torque Enabled");
				//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Torque Enabled", this.serialPort)) return false;
				
			}
			else {
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + ": Not valid Torque Enable value");
				return false;
			}
		} // OVERRIDE
		/// <summary>
		/// Enables Torque of the desired Servo
		/// </summary>
		/// <param name="enable">0 - Disable Torque. 1 - Enable Torque</param>
		/// <returns>Bool.</returns>
		public bool TorqueEnable(int id, int enable)
		{
			byte[] Cmd;
			if (enable == 0 || enable == 1)
			{
				Cmd = WriteServo(id, 24, 1, enable);
				return SendCommand(Cmd, "Set Torque Enabled");
				//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Torque Enabled", this.serialPort)) return false;
				//return true;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + ": Not valid Torque Enable value");
				return false;
			}
		}
		/// <summary>
		/// Gets Torque Enabled State
		/// </summary>
		/// <param name="enabled">0 - Torque Disabled; 1 - Torque Enabled</param>
		/// <returns>bool</returns>
		public virtual bool GetTorqueEnabled(out int enabled){
		
			if(!ReadData(24,out enabled))return false;
			return true;
	   }
		/// <summary>
		/// Gets Torque Enabled State
		/// </summary>
		/// <param name="enabled">0 - Torque Disabled; 1 - Torque Enabled</param>
		/// <returns>bool</returns>
		public virtual bool GetTorqueEnabled(int id, out int enabled)
		{

			if (!ReadData(id, 24, out enabled)) return false;
			return true;
		}
		
		/// <summary>
		/// Sets Baudrate
		/// </summary>
		/// <param name="baudrate">int BaudRate</param>
		/// <returns>bool</returns>
		public virtual bool SetBaudrate(double baudrate)
		{
			byte[] Cmd;
			int add;

			add = (int)Math.Round((2000000 / baudrate) - 1);
			if ((add == 1) || (add == 3) || (add == 4) || (add == 7) || (add == 9) || (add == 16) || (add == 34) || (add == 103) || (add == 207))
			{
				Cmd = WriteServo(this.id, 4, 1, add);
				return SendCommand(Cmd, "Set Baudrate");
				//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Baudrate", this.serialPort)) return false;
				//return true;
					  
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" Servo " + this.id + ": Incorrect BaudRate");
				return false;
			}

		} // OVERRIDE
		/// <summary>
		/// Sets Baudrate of the desired Servo
		/// </summary>
		/// <param name="baudrate">int BaudRate</param>
		/// <returns>bool</returns>
		public bool SetBaudrate(int id, double baudrate)
		{
			byte[] Cmd;
			int add;

			add = (int)Math.Round((2000000 / baudrate) - 1);
			if ((add == 1) || (add == 3) || (add == 4) || (add == 7) || (add == 9) || (add == 16) || (add == 34) || (add == 103) || (add == 207))
			{
				Cmd = WriteServo(id, 4, 1, add);
			return SendCommand(Cmd, "Set Baudrate");

				//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Baudrate", this.serialPort)) return false;
				//return true;
				
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" Servo " + id + ": Incorrect BaudRate");
				return false;
			}

		}

		/// <summary>
		/// Writes data in a register
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data">int data</param>
		/// <returns>bool</returns>
		public virtual bool WriteData(int address, int data)
		{
			byte[] Cmd;
			if (data >= 0 && data < 256)
			{
				Cmd = WriteServo(this.id, address, 1, data);
				return SendCommand(Cmd, "Write Servo");
				//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Write Servo", this.serialPort)) return false;
				//return true;
				
			}
			else {
				TextBoxStreamWriter.DefaultLog.WriteLine(" 	 Servo " + this.id + ": Incorrect Data");
				return false;
			}

		} // OVERRIDE
		/// <summary>
		/// Writes data in a register of the desired Servo
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data">int data</param>
		/// <returns>bool</returns>
		public bool WriteData(int id, int address, int data)
		{
			byte[] Cmd;

			if (data >= 0 && data < 256)
			{
				Cmd = WriteServo(id, address, 1, data);
				return SendCommand(Cmd, "Write Servo");
				//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Write Servo", this.serialPort)) return false;
				//return true;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" 	 Servo " + id + ": Incorrect Data");
				return false;
			}

		}
		/// <summary>
		/// Reads data from the register
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data">out int data</param>
		/// <returns>bool</returns>
		public virtual bool ReadData(int address, out int data)
		{

			byte[] Cmd;
			byte[] inCmd;
			data = 0;

			Cmd = ReadServo(this.id, address, 1);
			if (!SendAndReceiveCommand(Cmd, "Read Servo", out inCmd, 1)) return false;

			//if (!manipulatorStruct.serialPortManager.SendAndReceiveCommand(Cmd, "Read Servo", out inCmd, 7, this.serialPort)) return false;

			data = Convert.ToInt16(inCmd[5]);
			return true;
			
			
		} // OVERRIDE
		/// <summary>
		/// Reads data from the desired servo register
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data">out int data</param>
		/// <returns>bool</returns>
		public bool ReadData(int id, int address, out int data)
		{

			byte[] Cmd;
			byte[] inCmd;
			data = 0;
			Cmd = ReadServo(id, address, 1);
			if (!SendAndReceiveCommand(Cmd, "Read Servo", out inCmd, 1)) return false;
			//if (!manipulatorStruct.serialPortManager.SendAndReceiveCommand(Cmd, "Read Servo", out inCmd, 7, this.serialPort)) return false;
			data = Convert.ToInt16(inCmd[5]);
			return true;


		}


		/// <summary>
		/// Sets Maximum Torque to apply in bits
		/// </summary>
		/// <param name="torque">Max Torque [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetTorqueLimitBits(int torqueBits)
		{

			byte[] Cmd;
			int[] torqueBitsParts = new int[2];

			
			if (torqueBits <= 1023 && torqueBits >= 0)
			{
				torqueBitsParts[0] = torqueBits % 256;
				torqueBitsParts[1] = torqueBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo: " + this.id + " is out of torque range");
				return false;
			}
			Cmd = WriteServo(this.id, 34, 2, torqueBitsParts);
			return SendCommand(Cmd, "Set Torque Limit");

			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Torque Limit", this.serialPort)) return false;
			//return true;
			
		   
		} //OVERRIDE
		/// <summary>
		/// Sets Maximum Torque to apply of the desired Servo in bits
		/// </summary>
		/// <param name="torque">Max Torque [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool SetTorqueLimitBits(int id, int torqueBits)
		{

			byte[] Cmd;
			int[] torqueBitsParts = new int[2];


			if (torqueBits <= 1023 && torqueBits >= 0)
			{
				torqueBitsParts[0] = torqueBits % 256;
				torqueBitsParts[1] = torqueBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo: " + id + " is out of torque range");
				return false;
			}
			Cmd = WriteServo(id, 34, 2, torqueBitsParts);
			return SendCommand(Cmd, "Set Torque Limit");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Torque Limit", this.serialPort)) return false;
			//return true;

		}
		/// <summary>
		/// Sets Maximum Torque to apply in percentage %
		/// </summary>
		/// <param name="torque">torque [0,100][%]</param>
		/// <returns>bool</returns>
		public virtual bool SetTorqueLimitPercentage(double torque) {

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / 100);
			return SetTorqueLimitBits(torqueBits);

		} //OVERRIDE
		/// <summary>
		/// Sets Maximum Torque to apply of the desired Servo in percentage %
		/// </summary>
		/// <param name="torque">torque [0,100][%]</param>
		/// <returns>bool</returns>
		public bool SetTorqueLimitPercentage(int id, double torque)
		{

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / 100);
			return SetTorqueLimitBits(id, torqueBits);

		}
		/// <summary>
		/// Sets Maximum Torque to apply in kgf.cm
		/// </summary>
		/// <param name="torque">torque [kgf.c]</param>
		/// <returns></returns>
		public virtual bool SetTorqueLimit(double torque) {

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / this.maxServoTorque);
			return SetTorqueLimitBits(torqueBits);

		} // OVERRIDE

		/// <summary>
		/// Sets Maximum Torque to apply in bits
		/// </summary>
		/// <param name="torque">Max Torque [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool SetMaxTorqueBits(int torqueBits)
		{

			byte[] Cmd;
			int[] torqueBitsParts = new int[2];


			if (torqueBits <= 1023 && torqueBits >= 0)
			{
				torqueBitsParts[0] = torqueBits % 256;
				torqueBitsParts[1] = torqueBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo: " + this.id + " is out of torque range");
				return false;
			}
			Cmd = WriteServo(this.id, 14, 2, torqueBitsParts);
			return SendCommand(Cmd, "Set Max Torque");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Max Torque", this.serialPort)) return false;
			//return true;
			

		}  // OVERRIDE
		/// <summary>
		/// Sets Maximum Torque to apply of the desired Servo in bits
		/// </summary>
		/// <param name="torque">Max Torque [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool SetMaxTorqueBits(int id, int torqueBits)
		{

			byte[] Cmd;
			int[] torqueBitsParts = new int[2];


			if (torqueBits <= 1023 && torqueBits >= 0)
			{
				torqueBitsParts[0] = torqueBits % 256;
				torqueBitsParts[1] = torqueBits / 256;
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo: " + id + " is out of torque range");
				return false;
			}
			Cmd = WriteServo(id, 14, 2, torqueBitsParts);
			return SendCommand(Cmd, "Set Max Torque");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Max Torque", this.serialPort)) return false;
			//return true;

		}
		/// <summary>
		/// Sets Maximum Torque to apply in percentage %
		/// </summary>
		/// <param name="torque">torque [0,100][%]</param>
		/// <returns>bool</returns>
		public virtual bool SetMaxTorquePercentage(double torque)
		{

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / 100);
			return SetMaxTorqueBits(torqueBits);

		} // OVERRIDE
		/// <summary>
		/// Sets Maximum Torque to apply of the desired Servo  in percentage %
		/// </summary>
		/// <param name="torque">torque [0,100][%]</param>
		/// <returns>bool</returns>
		public bool SetMaxTorquePercentage(int id, double torque)
		{

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / 100);
			return SetMaxTorqueBits(id, torqueBits);

		}
		/// <summary>
		/// Sets Maximum Torque to apply in kgf.cm
		/// </summary>
		/// <param name="torque">torque [kgf.c]</param>
		/// <returns></returns>
		public virtual bool SetMaxTorque(double torque)
		{

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / this.maxServoTorque);
			return SetMaxTorqueBits(torqueBits);

		}  // OVERRIDE

		/// <summary>
		/// Sets Servo ID. Just one servo must be conected to serial port.
		/// </summary>
		/// <param name="id">ID desired</param>
		/// <returns>bool</returns>
		public bool SetIDBroadcast(int id)
		{

			byte[] Cmd;

			Cmd = WriteServo(254, 3, 1, id);
			return SendCommand(Cmd, "Set ID Broadcast");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set ID Broadcast", this.serialPort)) return false;
			//return true;
			
		}
		/// <summary>
		/// Sets Servo ID.
		/// </summary>
		/// <param name="id">ID desired</param>
		/// <returns>bool</returns>
		public virtual bool SetID(int id)
		{

			byte[] Cmd;

			Cmd = WriteServo(this.id, 3, 1, id);
			return SendCommand(Cmd, "Set ID");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set ID", this.serialPort)) return false;
			//return true;
			
		}
		/// <summary>
		/// Sets Servo ID.
		/// </summary>
		/// <param name="id1">Dynamixel ID</param>
		/// <param name="id2">Desired Dynamixel ID</param>
		/// <returns></returns>
		public bool SetID(int id1, int id2)
		{

			byte[] Cmd;

			Cmd = WriteServo(id1, 3, 1, id2);
			return SendCommand(Cmd, "Set ID");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set ID", this.serialPort)) return false;
			//return true;
			
		}

		/// <summary>
		/// Chages the Status Return Level
		/// 0 - Do not respond to any instruction
		/// 1 - Respond only to Read_Data instruction
		/// 2 - Respond to all instructions
		/// </summary>
		/// <param name="l">int [0,2]</param>
		/// <returns>bool</returns>
		public virtual bool SetStatusReturnLevel(int l)
		{
			byte[] Cmd;

			Cmd = WriteServo(this.id, 16, 1, l);
			return SendCommand(Cmd, "Set Status Return Level");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Status Return Level", this.serialPort)) return false;
			//return true;
			
		}
		/// <summary>
		/// Chages the Status Return Level of the desired Servo
		/// 0 - Do not respond to any instruction
		/// 1 - Respond only to Read_Data instruction
		/// 2 - Respond to all instructions
		/// </summary>
		/// <param name="l">int [0,2]</param>
		/// <returns>bool</returns>
		public bool SetStatusReturnLevel(int id, int l)
		{
			byte[] Cmd;

			Cmd = WriteServo(id, 16, 1, l);
			return SendCommand(Cmd, "Set Status Return Level");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Set Status Return Level", this.serialPort)) return false;
			//return true;
			
		}

		/// <summary>
		/// Gets Status Return Level
		/// </summary>
		/// <param name="enabled">0 - Torque Disabled; 1 - Torque Enabled</param>
		/// <returns>bool</returns>
		public virtual bool GetStatusReturnLevel(out int enabled)
		{

			if (!ReadData(16, out enabled)) return false;
			return true;
		}
		/// <summary>
		/// Gets Status Return Level of the desired Servo
		/// </summary>
		/// <param name="enabled">0 - Torque Disabled; 1 - Torque Enabled</param>
		/// <returns>bool</returns>
		public virtual bool GetStatusReturnLevel(int id, out int enabled)
		{

			if (!ReadData(id, 16, out enabled))
			{
				return false;
			}
			return true;
		}

		/// <summary>
		/// Sets CW Compliance slope of the desired servo.
		/// Compliance Slope can be defined as 7 levels, taking a valid position value: 
		/// 4 for a data value from 1 to 5
		/// 8 for a data value from 6 to 11
		/// 16 for a data value from 12 to 23
		/// 32 for a data value from 24 to 47
		/// 64 for a data value from 48 to 95
		/// 128 for a data value from 96 to 191
		/// 254 for a data value from 192 to 254
		/// </summary>
		/// <param name="slope"> Can take values from 1 to 254</param>
		/// <returns>bool</returns>
		public bool SetCWComplianceSlope(int id, int slope)
		{
			byte[] Cmd;

			if (slope >= 1 && slope <= 254)
			{
				if (slope > 0 && slope < 6) slope = 4;
				if (slope > 5 && slope < 12) slope = 8;
				if (slope > 11 && slope < 24) slope = 16;
				if (slope > 23 && slope < 48) slope = 32;
				if (slope > 47 && slope < 96) slope = 64;
				if (slope > 95 && slope < 192) slope = 128;
				if (slope > 191 && slope < 255) slope = 254;

				Cmd = WriteServo(id, 28, 1, slope);
				return SendCommand(Cmd, "Set Compliance Slope");

			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" Servo " + id + ": Compliance Slope is out of rate");
				return false;
			}
		}
		/// <summary>
		/// Sets CW Compliance slope of the desired servo.
		/// Compliance Slope can be defined as 7 levels, taking a valid position value: 
		/// 4 for a data value from 1 to 5
		/// 8 for a data value from 6 to 11
		/// 16 for a data value from 12 to 23
		/// 32 for a data value from 24 to 47
		/// 64 for a data value from 48 to 95
		/// 128 for a data value from 96 to 191
		/// 254 for a data value from 192 to 254
		/// </summary>
		/// <param name="slope"> Can take values from 1 to 254</param>
		/// <returns>bool</returns>
		public bool SetCCWComplianceSlope(int id, int slope)
		{
			byte[] Cmd;

			if (slope >= 1 && slope <= 254)
			{
				if (slope > 0 && slope < 6) slope = 4;
				if (slope > 5 && slope < 12) slope = 8;
				if (slope > 11 && slope < 24) slope = 16;
				if (slope > 23 && slope < 48) slope = 32;
				if (slope > 47 && slope < 96) slope = 64;
				if (slope > 95 && slope < 192) slope = 128;
				if (slope > 191 && slope < 255) slope = 254;

				Cmd = WriteServo(id, 29, 1, slope);
				return SendCommand(Cmd, "Set Compliance Slope");

			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" Servo " + id + ": Compliance Slope is out of rate");
				return false;
			}
		}

		public bool SetCWComplianceSlope(int slope)
		{
			byte[] Cmd;

			if (slope >= 1 && slope <= 254)
			{
				if (slope > 0 && slope < 6) slope = 4;
				if (slope > 5 && slope < 12) slope = 8;
				if (slope > 11 && slope < 24) slope = 16;
				if (slope > 23 && slope < 48) slope = 32;
				if (slope > 47 && slope < 96) slope = 64;
				if (slope > 95 && slope < 192) slope = 128;
				if (slope > 191 && slope < 255) slope = 254;

				Cmd = WriteServo(this.id, 28, 1, slope);
				return SendCommand(Cmd, "Set Compliance Slope");

			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" Servo " + this.id + ": Compliance Slope is out of rate");
				return false;
			}
		}
		/// <summary>
		/// Sets CW Compliance slope of the desired servo.
		/// Compliance Slope can be defined as 7 levels, taking a valid position value: 
		/// 4 for a data value from 1 to 5
		/// 8 for a data value from 6 to 11
		/// 16 for a data value from 12 to 23
		/// 32 for a data value from 24 to 47
		/// 64 for a data value from 48 to 95
		/// 128 for a data value from 96 to 191
		/// 254 for a data value from 192 to 254
		/// </summary>
		/// <param name="slope"> Can take values from 1 to 254</param>
		/// <returns>bool</returns>
		public bool SetCCWComplianceSlope(int slope)
		{
			byte[] Cmd;

			if (slope >= 1 && slope <= 254)
			{
				if (slope > 0 && slope < 6) slope = 4;
				if (slope > 5 && slope < 12) slope = 8;
				if (slope > 11 && slope < 24) slope = 16;
				if (slope > 23 && slope < 48) slope = 32;
				if (slope > 47 && slope < 96) slope = 64;
				if (slope > 95 && slope < 192) slope = 128;
				if (slope > 191 && slope < 255) slope = 254;

				Cmd = WriteServo(this.id, 29, 1, slope);
				return SendCommand(Cmd, "Set Compliance Slope");

			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(" Servo " + this.id + ": Compliance Slope is out of rate");
				return false;
			}
		}
		#endregion

		#region Status

		/// <summary>
		/// Gets Servo Load in Bits
		/// </summary>
		/// <param name="data">out int Speed [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool GetLoadBits(out int loadBits,out int sign)
		{
			byte[] Cmd = new byte[9];
			byte[] inCmd = new byte[8];
			sign = 0;
			loadBits = 0;
			Cmd = ReadServo(this.id, 40, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Load", out inCmd, 2)) return false;

			//if (!SendCommand(Cmd, "Get Load"))
			//{
			//    loadBits = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(this.id, out inCmd, "Get Load", 2))
			//{
			//    loadBits = 0;
			//    return false;
			//}

			loadBits = (((inCmd[5]) + ((inCmd[6]) * 256)));
			if ((loadBits & 1024) == 1024)
			{
				sign = -1;
			}
			if ((loadBits & 1024) == 0)
			{
				sign = 1;
			}

			if (this.cw) sign = sign * -1;

			loadBits = loadBits & 511;
			return true;

		}  // OVERRIDE
		/// <summary>
		/// Gets Load of the desired Servo in Bits
		/// </summary>
		/// <param name="id">Desired Dynamixel ID</param>
		/// <param name="data">out int Speed [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool GetLoadBits(int id, out int loadBits,out int sign)
		{
			byte[] Cmd = new byte[9];
			byte[] inCmd = new byte[8];
			sign = 0;
			loadBits = 0;
			Cmd = ReadServo(id, 40, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Load", out inCmd, 2)) return false;

			loadBits = (((inCmd[5]) + ((inCmd[6]) * 256)));

						
			if ((loadBits & 1024) == 1024)
			{
				sign = -1;
			}
			if ((loadBits & 1024) == 0)
			{
				sign = 1;
			}

			if (this.cw) sign = sign * -1;

			loadBits = loadBits & 511;
			return true;

		}
		/// <summary>
		/// Gets current Load
		/// </summary>
		/// <param name="load">double Load [0,MaxServoTorque][kgf.cm]</param>
		/// <returns>bool</returns>
		public virtual bool GetLoad(out double load)
		{
			int loadBits;
			int sign;
			if (!GetLoadBits(out loadBits, out sign))
			{
				load = 0;
				return false;
			}
		 
			load = sign * (loadBits * this.maxServoTorque) / 511;
			return true;

		}  // OVERRIDE
		/// <summary>
		/// Gets Current Servo Load in percentage %
		/// </summary>
		/// <param name="load">double Load [0,100][%]</param>
		/// <returns>bool</returns>
		public virtual bool GetLoadPercentage(out double loadPercentage)
		{
			int loadBits;
			int sign;
			if (!GetLoadBits(out loadBits, out sign))
			{
				loadPercentage = 0;
				return false;
			}

			loadPercentage = sign * (loadBits * 100) / 511;
			return true;
		}   // OVERRIDE
		/// <summary>
		/// Gets Current Servo Load in percentage %
		/// </summary>
		/// <param name="id">Desired Dynamixel ID</param>
		/// <param name="load">double Load [0,100][%]</param>
		/// <returns>bool</returns>
		public bool GetLoadPercentage(int id, out double loadPercentage)
		{
			int loadBits;
			int sign;
			if (!GetLoadBits(id, out loadBits,out sign))
			{
				loadPercentage = 0;
				return false;
			}

			loadPercentage = sign * (loadBits * 100) / 511;
			return true;
		}

		/// <summary>
		/// Gets Torque Limit in bits
		/// </summary>
		/// <param name="torqueLimit">int Torque Limit [0,1023][bits]</param>
		/// <returns>bool</returns>
		public virtual bool GetTorqueLimitBits(out int torqueLimit)
		{
			byte[] Cmd = new byte[9];
			byte[] inCmd = new byte[8];
			torqueLimit = 0;
			Cmd = ReadServo(this.id, 34, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Torque Limit", out inCmd, 2)) return false;

			//if (!SendCommand(Cmd, "Get Torque Limit"))
			//{
			//    torqueLimit = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(this.id, out inCmd, "Get Torque Limit", 2))
			//{
			//    torqueLimit = 0;
			//    return false;
			//}

			torqueLimit = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));
			return true;


		}  // OVERRIDE
		/// <summary>
		/// Gets Torque Limit in bits
		/// </summary>
		/// <param name="id">Desired Dynamixel ID</param>
		/// <param name="torqueLimit">int Torque Limit [0,1023][bits]</param>
		/// <returns>bool</returns>
		public bool GetTorqueLimitBits(int id, out int torqueLimit)
		{
			byte[] Cmd = new byte[9];
			byte[] inCmd = new byte[8];
			torqueLimit = 0;
			Cmd = ReadServo(id, 34, 2);

			if (!SendAndReceiveCommand(Cmd, "Get Torque Limit", out inCmd, 2)) return false;

			//if (!SendCommand(Cmd, "Get Torque Limit"))
			//{
			//    torqueLimit = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(id, out inCmd, "Get Torque Limit", 2))
			//{
			//    torqueLimit = 0;
			//    return false;
			//}

			torqueLimit = ((Convert.ToInt16(inCmd[5]) + (Convert.ToInt16(inCmd[6]) * 256)));
			return true;


		}
		/// <summary>
		/// Gets Torque Limit.
		/// </summary>
		/// <param name="torqueLimit">double Torque Limit [0,MaxServoTorque][kgf.cm]</param>
		/// <returns>bool</returns>
		public virtual bool GetTorqueLimit(out double torqueLimit)
		{
			int torqueLimitBits;
			if (!GetTorqueLimitBits(out torqueLimitBits))
			{
				torqueLimit = 0;
				return false;
			}

			torqueLimit = (torqueLimitBits) * (this.maxServoTorque / 1023);
			return true;


		}   // OVERRIDE
		/// <summary>
		/// Gets Torque Limit in percentage.
		/// </summary>
		/// <param name="torqueLimit">double Torque Limit [0,100][%]</param>
		/// <returns>bool</returns>
		public virtual bool GetTorqueLimitPercentage(out double torqueLimit)
		{
			int torqueLimitBits;
			if (!GetTorqueLimitBits(out torqueLimitBits))
			{
				torqueLimit = 0;
				return false;
			}

			torqueLimit = (torqueLimitBits) * (100.0 / 1023.0);
			return true;


		}  // OVERRIDE
		/// <summary>
		/// Gets Torque Limit of the desired Sero in percentage.
		/// </summary>
		/// <param name="id">Desired Dynamixel ID</param>
		/// <param name="torqueLimit">double Torque Limit [0,100][%]</param>
		/// <returns>bool</returns>
		public bool GetTorqueLimitPercentage(int id, out double torqueLimitPercentage)
		{
			int torqueLimitBits;
			if (!GetTorqueLimitBits(id, out torqueLimitBits))
			{
				torqueLimitPercentage = 0;
				return false;
			}

			torqueLimitPercentage = (torqueLimitBits) * (100.0 / 1023.0);
			return true;


		}

		
		
		/// <summary>
		/// Gets Serbo Voltage
		/// </summary>
		/// <param name="voltage">double voltage [V]</param>
		/// <returns>bool</returns>
		public virtual bool GetVoltage(out double voltage)
		{

			byte[] Cmd = new byte[8];
			byte[] inCmd = new byte[7];
			voltage = 0;
			Cmd = ReadServo(this.id, 42, 1);

			if (!SendAndReceiveCommand(Cmd, "Get Voltage", out inCmd, 1)) return false;

			//if (!SendCommand(Cmd, "Get Voltage"))
			//{
			//    voltage = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(this.id, out inCmd, "Get Voltage", 1))
			//{
			//    voltage = 0;
			//    return false;
			//}

			voltage = Convert.ToDouble(inCmd[5]) / 10;
			return true;

		} // OVERRIDE
		/// <summary>
		/// Gets Voltage of the desired Servo
		/// </summary>
		/// <param name="id">Desired Dynamixel ID</param>
		/// <param name="voltage">double voltage [V]</param>
		/// <returns>bool</returns>
		public bool GetVoltage(int id, out double voltage)
		{

			byte[] Cmd = new byte[8];
			byte[] inCmd = new byte[7];

			Cmd = ReadServo(id, 42, 1);
			voltage = 0;
			if (!SendAndReceiveCommand(Cmd, "Get Voltage", out inCmd, 1)) return false;


			//if (!SendCommand(Cmd, "Get Voltage"))
			//{
			//    voltage = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(id, out inCmd, "Get Voltage", 1))
			//{
			//    voltage = 0;
			//    return false;
			//}

			voltage = Convert.ToDouble(inCmd[5]) / 10;
			return true;

		}
		
		/// <summary>
		/// Gets Servo Temperature
		/// </summary>
		/// <param name="temp">double temperature [°C]</param>
		/// <returns>bol</returns>
		public virtual bool GetTemp(out double temp)
		{

			byte[] Cmd = new byte[8];
			byte[] inCmd = new byte[7];

			Cmd = ReadServo(this.id, 43, 1);
			temp = 0;
			if (!SendAndReceiveCommand(Cmd, "Get Temperature", out inCmd, 1)) return false;


			//if (!SendCommand(Cmd, "Get Voltage"))
			//{
			//    temp = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(this.id, out inCmd, "Get Voltage", 1))
			//{
			//    temp = 0;
			//    return false;
			//}

				temp = Convert.ToDouble(inCmd[5]);
				return true;
			
		}
		/// <summary>
		/// Gets Temperature of the desired Servo
		/// </summary>
		/// <param name="temp">double temperature [°C]</param>
		/// <returns>bol</returns>
		public bool GetTemp(int id, out double temp)
		{

			byte[] Cmd = new byte[8];
			byte[] inCmd = new byte[7];

			Cmd = ReadServo(id, 43, 1);
			temp = 0;
			if (!SendAndReceiveCommand(Cmd, "Get Temperature", out inCmd, 1)) return false;

			//if (!SendCommand(Cmd, "Get Voltage"))
			//{
			//    temp = 0;
			//    return false;
			//}
			//if (!ReceiveCommand(id, out inCmd, "Get Voltage", 1))
			//{
			//    temp = 0;
			//    return false;
			//}

			temp = Convert.ToDouble(inCmd[5]);
			return true;

		}

		

		#endregion

		#region StatusInternal

		/// <summary>
		/// Sends Ping and get all errors in errors bool array internal variable.
		/// </summary>
		/// <returns>bool</returns>
		public virtual bool Ping()
		{
			if (!Ping(out errors)) return false;
			return true;
		}
		/// <summary>
		/// Gets Servo Torque Enable State in torqueEnabled internal variable
		/// </summary>
		/// <returns>bool</returns>
		public virtual bool GetTorqueEnabled() {
			int e=0;
			if (!GetTorqueEnabled(out e)) return false;
			if (e == 0) torqueEnabled = false;
			if (e == 1) torqueEnabled = true;
			return true;
		}
		/// <summary>
		/// Gets Servo Torque Limit in Percentage in torqueLimitPerc internal variable
		/// </summary>
		/// <returns>bool</returns>
		public virtual bool GetTorqueLimitPercentage() {

			if (!GetTorqueLimitPercentage(out torqueLimitPerc)) return false;
			return true;
		}
		/// <summary>
		/// Executes Ping, GetTorqueEnabled,GetTorqueLimitPErcentage,GetTemp,GetVoltage,GetLoad
		/// </summary>
		/// <returns>bool</returns>
 		public virtual bool GlobalStatus() {
			
			Ping();
			GetTorqueEnabled();
			GetTorqueLimitPercentage();
			GetTemp();
			GetVoltage();
			GetLoad();

			return true;
		
		}
		/// <summary>
		/// Gets Servo Temperature in temp variable
		/// </summary>
		/// <returns></returns>
		public virtual bool GetTemp() {

			if (!GetTemp(out temp)) return false;
			return true;
		
		}
		/// <summary>
		/// Gets Servo Voltage in voltage variable
		/// </summary>
		/// <returns></returns>
		public virtual bool GetVoltage() {
			double v;
			if (!GetVoltage(out v)) return false;
			Voltage = v;
			return true;
			
		}
		/// <summary>
		/// Gets Servo Current Load in a load variable
		/// </summary>
		/// <returns></returns>
		public virtual bool GetLoad() {
			if (!GetLoadPercentage(out load)) return false;
			return true;
		}
		#endregion

		#endregion

		#region BASIC INSTRUCTIONS
		/// <summary>
		/// Makes the Byte array to Write Instruction
		/// </summary>
		/// <param name="id">int Dynamixel ID</param>
		/// <param name="address">int Address of Dynamixel Memory</param>
		/// <param name="numberOfData">int Number of Data to Write</param>
		/// <param name="data">int[] int Array of Data to Write </param>
		/// <returns>byte[] Byte Array of Write Instruction</returns>
		protected byte[] WriteServo(int id, int address, int numberOfData, int[] data)
		{

			int dataLength;
			dataLength = (numberOfData + 1) + 6;
			byte[] Cmd = new byte[dataLength];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);				// Dynamixel ID
			Cmd[3] = Convert.ToByte(numberOfData + 3);  // Instruction Length
			Cmd[4] = 3;								 // Instruction
			Cmd[5] = Convert.ToByte(address);		   // Star Address of Data to write
			for (int i = 0; i < numberOfData; i++)
			{
				Cmd[i + 6] = Convert.ToByte(data[i]);
			}
			Cmd[dataLength - 1] = CheckSum(Cmd);

			return Cmd;

		} 
		/// <summary>
		/// Makes the Byte array to Write Instruction
		/// </summary>
		/// <param name="id">int Dynamixel ID</param>
		/// <param name="address">int Address of Dynamixel Memory</param>
		/// <param name="numberOfData">int Number of Data to Write = 1</param>
		/// <param name="data">int Data to Write </param>
		/// <returns>byte[] Byte Array of Write Instruction</returns>
		protected byte[] WriteServo(int id, int address, int numberOfData, int data)
		{

			int dataLength;
			dataLength = (numberOfData + 1) + 6;
			byte[] Cmd = new byte[dataLength];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);				// Dynamixel ID
			Cmd[3] = Convert.ToByte(numberOfData + 3);  // Instruction Length
			Cmd[4] = 3;								 // Instruction
			Cmd[5] = Convert.ToByte(address);		   // Star Address of Data to write
			Cmd[6] = Convert.ToByte(data);
			Cmd[7] = CheckSum(Cmd);

			return Cmd;

		}
		/// <summary>
		/// Makes the Byte array to Read instruction
		/// </summary>
		/// <param name="id">int Dynamixel ID</param>
		/// <param name="address">int Address of Dynamixel Memory</param>
		/// <param name="numberOfData">int Number of Data to Read</param>
		/// <returns>byte[] Byte Array of Read Instruction</returns>
		protected byte[] ReadServo(int id, int address, int numberOfData)
		{

			int dataLength;
			dataLength = 8;
			byte[] Cmd = new byte[dataLength];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);				// Dynamixel ID
			Cmd[3] = 4;								 // Instruction Length
			Cmd[4] = 2;								 // Instruction
			Cmd[5] = Convert.ToByte(address);		   // Star Address of data to be read
			Cmd[6] = Convert.ToByte(numberOfData);	  // Length of Data to be read
			Cmd[7] = CheckSum(Cmd);

			return Cmd;
		}
		/// <summary>
		/// Makes the Byte array to Reg Write Instruction
		/// </summary>
		/// <param name="id">int Dynamixel ID</param>
		/// <param name="address">int Address of Dynamixel Memory</param>
		/// <param name="numberOfData">int Number of Data to Write</param>
		/// <param name="data">int[] int Array of Data to Write </param>
		/// <returns>byte[] Byte Array of Reg Write Instruction</returns>
		protected byte[] RegWriteServo(int id, int address, int numberOfData, int[] data)
		{

			int dataLength;
			dataLength = (numberOfData + 1) + 6;
			byte[] Cmd = new byte[dataLength];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);				// Dynamixel ID
			Cmd[3] = Convert.ToByte(numberOfData + 3);  // Instruction Length
			Cmd[4] = 4;								 // Instruction
			Cmd[5] = Convert.ToByte(address);		   // Star Address of Data to write
			for (int i = 0; i < numberOfData; i++)
			{
				Cmd[i + 6] = Convert.ToByte(data[i]);
			}
			Cmd[dataLength - 1] = CheckSum(Cmd);

			return Cmd;

		}

		/// <summary>
		/// Executes buffer instruction.
		/// </summary>
		/// <returns>bool</returns>
		public bool Execute()
		{
			byte[] Cmd = new byte[6];

			Cmd[0] = 255; // 
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(this.id);		// Dynamixel ID
			Cmd[3] = 2;						 // Instruction Length 
			Cmd[4] = 5;						 // Instrucción Action  
			Cmd[5] = CheckSum(Cmd);

			return SendCommand(Cmd, "Execute");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Execute", this.serialPort)) return false;
			//return true;
			

		}
		/// <summary>
		/// Executes buffer instruction of desired ID
		/// </summary>
		/// <param name="id">Dynamixel ID</param>
		/// <returns>bool</returns>
		public bool Execute(int id)
		{
			byte[] Cmd = new byte[6];

			Cmd[0] = 255; // 
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);		// Dynamixel ID
			Cmd[3] = 2;						 // Instruction Length 
			Cmd[4] = 5;						 // Instrucción Action  
			Cmd[5] = CheckSum(Cmd);

			return SendCommand(Cmd, "Execute");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Execute", this.serialPort)) return false;
			//return true;

		}
		/// <summary>
		/// Execute Buffer Instruction. Broadcast
		/// </summary>
		/// <returns>bool</returns>
		public bool ExecuteBroadcast()
		{
			byte[] Cmd = new byte[6];

			Cmd[0] = 255;
			Cmd[1] = 255;
			Cmd[2] = 254;	   // BroadCast ID
			Cmd[3] = 2;		 // Instruction Length
			Cmd[4] = 5;		 // Instrucción Action
			Cmd[5] = 250;	   // CheckSum for Broadcast

			return SendCommand(Cmd, "Execute Broadcast");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Execute Broadcast", this.serialPort)) return false;
			//rrturn true;


		}

	
		/// <summary>
		/// Show sings of life ja
		/// </summary>
		/// <returns>bool</returns>
		public bool Ping(out bool[] errors)
		{
			byte[] inCmd;
			byte[] Cmd = new byte[6];
			errors = new bool[7];
            for (int i = 0; i < errors.Length; i++) { errors[i] = true; }
			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(this.id);//Convert.ToByte(id); // ID de motor
			Cmd[3] = 2;  // Longitud de instrucción (N+3) 
			Cmd[4] = 1;  // Instrucción Ping
			Cmd[5] = CheckSum(Cmd);

			//return SendCommand(Cmd, "Ping");
			if (!SendAndReceiveCommand(Cmd, "Ping", out inCmd, 0)) return false;
			
			errors = ErrorParse(inCmd[4]);
			return true;

			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Ping", this.serialPort)) return false;
			//return true;
			
		}
		/// <summary>
		/// Show sings of life of desired Dynamiexl ID ja
		/// </summary>
		/// <param name="id">Dynamixel ID</param>
		/// <returns></returns>
		public bool Ping(int id,out bool[] errors)
		{

			byte[] inCmd;
			byte[] Cmd = new byte[6];
			errors = new bool[7];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);//Convert.ToByte(id); // ID de motor
			Cmd[3] = 2;  // Longitud de instrucción (N+3) 
			Cmd[4] = 1;  // Instrucción Ping
			Cmd[5] = CheckSum(Cmd);


			if (!SendAndReceiveCommand(Cmd, "Ping", out inCmd, 0)) return false;

			errors = ErrorParse(inCmd[4]);
			return true;

			//return SendCommand(Cmd, "Ping");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Ping", this.serialPort)) return false;
			//return true;
		}

        /// <summary>
        /// Execute Ping without considering errors.
        /// </summary>
        /// <param name="id">int servo ID</param>
        /// <returns>bool if executed</returns>
        public bool Ping(int id)
        {
            bool[] errors;
            bool ok = Ping(id, out errors);
            return ok;
        }
		/// <summary>
		/// Restore the condition of the Contorl Table of the Dynamixel back to Factory Default
		/// </summary>
		/// <returns>bool</returns>
		public bool Reset()
		{
			byte[] Cmd = new byte[6];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(this.id);//Convert.ToByte(id); // ID de motor
			Cmd[3] = 2;  // Longitud de instrucción (N+3)
			Cmd[4] = 6;  // Instrucción Reset
			Cmd[5] = CheckSum(Cmd);

			return SendCommand(Cmd, "Reset");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Reset", this.serialPort)) return false;
			//return true;
			
		}
		/// <summary>
		/// Restore the condition of the Control Table of the desired Dynamixel back to Factory Default
		/// </summary>
		/// <returns>bool</returns>
		public bool Reset(int id)
		{
			byte[] Cmd = new byte[6];

			Cmd[0] = 255; // inicio Instrucción
			Cmd[1] = 255;
			Cmd[2] = Convert.ToByte(id);//Convert.ToByte(id); // ID de motor
			Cmd[3] = 2;  // Longitud de instrucción (N+3)
			Cmd[4] = 6;  // Instrucción Reset
			Cmd[5] = CheckSum(Cmd);

			return SendCommand(Cmd, "Reset");
			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Reset", this.serialPort)) return false;
			//return true;
		}
		
		/// <summary>
		/// Calculates the CheckSum for any Method
		/// </summary>
		/// <param name="Cmd">Bytes array of N-1 (N := number of arguments of the instruction)</param>
		/// <returns>Byte N (CheckSum</returns>
		/// 
		protected byte CheckSum(byte[] Cmd)
		{
			byte sum = 0;
			byte checkSum;
			unchecked
			{
				for (int i = 2; i < (Cmd.Length - 1); i++)
				{
					sum = (byte)(sum + Cmd[i]);
				}
			}
			checkSum = (byte)(255 - sum);

			return checkSum;
		}

		/// <summary>
		/// Sends any kind of command.
		/// </summary>
		/// <param name="cmd">byte[] Instruction Packet</param>
		/// <param name="nameCmd">Command Name for Error reporting</param>
		/// <returns></returns>
		protected bool SendCommand(byte[] cmd, string cmdName)
		{
			string id = cmd[2].ToString();

			if (!serialPortManager.SendCommand(this.dynamixelType,cmd))
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " +id + ": " + cmdName + " Error");
				 return false;
			}
				return true;
			}
		/// <summary>
		/// Receive Data and Reports Error
		/// </summary>
		/// <param name="id">Dynamixel ID</param>
		/// <param name="nameCmd">Command Name for Error reporting</param>
		/// <returns>bool</returns>
		protected bool ReceiveCommand(int id, out byte[] inCmd, string nameCmd, int dataLength)
		{

			if (!serialPortManager.ReceiveCommand(this.dynamixelType, dataLength, out inCmd))
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + ": " + nameCmd + " Error");

				return false;
			}
			return true;
		}

		protected bool SendAndReceiveCommand(byte[] cmd, string cmdName, out byte[] inCmd, int dataLength) {
			
			int id = cmd[2];
			inCmd = null;

			if (!serialPortManager.SendReceiveCommand(this.dynamixelType, cmd, dataLength, out inCmd))
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + ": " + cmdName + " Error");
				return false;
			}

			return true;
		}

	

		 ///<summary>
		 ///Parse Error Byte from the Status Package
		 ///</summary>
		 ///<param name="Cmd4">Status Package Error Byte</param>
		 ///<returns>Bool Array. Every Element of the array is a especific error</returns>
		protected bool[] ErrorParse(byte Cmd4)
		{
			bool[] error;
			error = new bool[7];

			error[0] = Convert.ToBoolean(Cmd4 & 1);
			error[1] = Convert.ToBoolean(Cmd4 & 2);
			error[2] = Convert.ToBoolean(Cmd4 & 4);
			error[3] = Convert.ToBoolean(Cmd4 & 8);
			error[4] = Convert.ToBoolean(Cmd4 & 16);
			error[5] = Convert.ToBoolean(Cmd4 & 32);
			error[6] = Convert.ToBoolean(Cmd4 & 64);

			return error;
		}
		 ///<summary>
		 ///Converts the parsed Bool Array to String Error Data
		 ///</summary>
		 ///<param name="errors">Error Bool Array</param>
		 ///<returns>Complete String of Errors</returns>
		protected string ErrorToString(bool[] errors)
		{

			string errorString = "";
			string inputVoltage, angleLimit, overheating, range, checksum, overload, instruction;
			inputVoltage = "";
			angleLimit = "";
			overheating = "";
			range = "";
			checksum = "";
			overload = "";
			instruction = "";

			if (errors[0]) inputVoltage = "Input Voltage ";
			if (errors[1]) angleLimit = "Angle Limit ";
			if (errors[2]) overheating = "Overheating ";
			if (errors[3]) range = "Range ";
			if (errors[4]) checksum = "Checksum ";
			if (errors[5]) overload = "Overload ";
			if (errors[6]) instruction = "Instruction ";



			return errorString = inputVoltage + angleLimit + overheating + range + checksum + overload + instruction;



		}
		/// <summary>
		/// After an error has ocurred, Some kind of fixes must be done
		/// </summary>
		/// <param name="errors">Bool array of errors</param>
		/// <returns>bool</returns>
		protected bool ErrorCorrection(bool[] errors)
		{

			if (errors[0]) TextBoxStreamWriter.DefaultLog.WriteLine("Input Error Correction"); // Input Voltage 
			if (errors[1]) TextBoxStreamWriter.DefaultLog.WriteLine("Angle Limit Error Correction"); ;	 // Angle Limit
			if (errors[2]) TextBoxStreamWriter.DefaultLog.WriteLine("Overheating Error Correction"); ;	// Overheating
			if (errors[3]) TextBoxStreamWriter.DefaultLog.WriteLine("Range Error Correction"); ;				// Range
			if (errors[4]) TextBoxStreamWriter.DefaultLog.WriteLine("Checksum Error Correction"); ;		  // Checksum error
			if (errors[5]) TextBoxStreamWriter.DefaultLog.WriteLine("Overload Error Correction"); ;		  // Overload
			if (errors[6]) TextBoxStreamWriter.DefaultLog.WriteLine("Instruction Error Correction"); ;	// Instruction Error

			return true;
		}
		#endregion

		#region MathBasics
		/// <summary>
		/// Converts Degrees to Radians
		/// </summary>
		/// <param name="degrees">Degrees</param>
		/// <returns>Radians</returns>
		public double ToRadians(double degrees) {
			double radians;
			radians = degrees * (Math.PI / 180);
			return radians;
		}
		/// <summary>
		/// Converts Radians to Degrees
		/// </summary>
		/// <param name="radians">Radians</param>
		/// <returns>Degrees</returns>
		public double ToDegrees(double radians) {
			double degrees;
			degrees = radians * (180 / Math.PI);
			return degrees;
		}
		#endregion

	}
	
}
