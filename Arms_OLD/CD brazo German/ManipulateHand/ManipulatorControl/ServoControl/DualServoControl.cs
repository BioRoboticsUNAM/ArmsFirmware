using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.IO.Ports;
using Robotics.Controls;

namespace ManipulatorControl
{
	class DualServoControl: ServoControl
	{
		#region VARIABLES

		public readonly bool aligned;   
		
		protected bool[] errors1;
		protected bool[] errors2;
		//protected bool[] errors;
		//protected bool pingOK;
		//protected bool torqueEnabled;
		//protected double torqueLimitPerc;
		//protected int servoStatus;
		//protected double temp;
		//protected double load;
		//protected double voltage;


		#endregion

		#region CONSTUCTOR
		/// <summary>
		/// Creates Dual Servo Object
		/// </summary>
		/// <param name="dynamixelType">Determines Dynamixel Type for speed and torque calculation</param>
		/// <param name="id">Dynamixel ID</param>
		/// <param name="ceroPosition">Place of the cero reference. Use normal reference  180° <-- --> 0°</param>
		/// <param name="cw">ClockWise motion</param>
		/// <param name="aligned">Servos are placed face to face or aligned</param>
		/// <param name="puerto">Serial Port to use</param>
		public DualServoControl(ServoType dynamixelType, int id,double ceroPosition,  bool cw, bool aligned, SerialPortManager serialPortManager)
			: base(dynamixelType, id, ceroPosition, cw, serialPortManager)
		{

			this.aligned = aligned;

			switch (dynamixelType)
			{
				case ServoType.RX64:

					voltage = 18;
					this.maxServoSpeed = (voltage * 0.366616119) + 0.070957958;	  // From DataSheet
					this.maxServoTorque = ((voltage * 4.266666667) + 0.4)*2;			  // From Datasheet
					
					break;
				case ServoType.AX12:
					voltage = 10;
					this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
					this.maxServoTorque = ((voltage * 1.5) + 1.5)*2;			  // From Datasheet
					break;

				case ServoType.EX106:
					voltage = 18;
					this.maxServoSpeed = (voltage * 0.424115432) - 0.607898786;	  // From DataSheet
					this.maxServoTorque = ((voltage * 5.945945946) - 4)*2;			  // From Datasheet
					break;
				default:
					voltage = 10;
					this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
					this.maxServoTorque = ((voltage * 1.5) + 1.5)*2;			  // From Datasheet
					break;
			}

			  dualServo = true;
			  errors1 = new bool[7];
			  errors2 = new bool[7];
			  errors = new bool[7];
             // for (int i = 0; i < errors.Length; i++) { errors[i] = true; }
              for (int i = 0; i < errors1.Length; i++) { errors1[i] = true; }
              for (int i = 0; i < errors2.Length; i++) { errors2[i] = true; }
		}
		
		

		#endregion

		#region PROPERTIES
		public override bool PingOK
		{

			get
			{
				
				pingOK = false;
				for (int i = 0; i < errors.Length; i++)
				{
					pingOK = pingOK || errors1[i]; ;
				}
				for (int i = 0; i < errors.Length; i++)
				{
					pingOK = pingOK || errors2[i]; ;
				}
				
				return !pingOK;


			}


		}
		public bool[] Errors1
		{
			get
			{			//Ping();
				return errors1;

			}

		}
		public bool[] Errors2
		{
			get
			{			//Ping();
				return errors2;

			}

		}
		public override bool[] Errors {

			get {
				for (int i = 0; i < errors.Length; i++) {
					errors[i] = errors1[i] || errors2[i];
				}
				return errors;
			}
		
		}
		public override double TorqueLimitPerc
		{
			get
			{
				return torqueLimitPerc;
			}
		}
		public override bool TorqueEnabled
		{
			get
			{
					return torqueEnabled;
			}
		}
		public override int ServoStatus
		{
			get
			{
				//if (!GlobalStatus()) return 0;
				if (!PingOK)
				{
					servoStatus = 0;
					return servoStatus;
				}
				if ((TorqueEnabled) && (TorqueLimitPerc > 80))
				{
					servoStatus = 2;
					return servoStatus;
				}
				else
				{
					servoStatus = 1;
					return servoStatus;
				}



			}

		}
		public override bool IsAlive
		{
			get
			{
				//if (!GlobalStatus()) return false;

				if (servoStatus == 1)
				{
					return true;
				}
				else {
					return false;
				}
			}
		}
		public override double Voltage
		{
			get
			{
				return voltage;
			}
			set
			{
				voltage = value;
				switch (dynamixelType)
				{
					case ServoType.RX64:

						this.maxServoSpeed = (voltage * 0.366616119) + 0.070957958;	  // From DataSheet
						this.maxServoTorque = ((voltage * 4.266666667) + 0.4) *2;			  // From Datasheet
						break;
					case ServoType.AX12:

						this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
						this.maxServoTorque = ((voltage * 1.5) + 1.5) *2;			  // From Datasheet
						break;
					default:

						this.maxServoSpeed = (voltage * 0.48330565) - 3.356657047;	  // From DataSheet
						this.maxServoTorque = ((voltage * 1.5) + 1.5)*2;			  // From Datasheet
						break;

				}
			}
		}
		public override double Load
		{

			get { return load; }
			set { load = value; }
		}
		public override double Temp
		{

			get { return temp; }
			set
			{
				temp = value;
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
		public override bool SetPositionBits(int thetaBits)
		{
			int inverseThetaBits;

			if (this.aligned)
			{
				if (!SetPositionBitsWait(this.id, thetaBits))  return false;
				if (!SetPositionBitsWait(this.id + 10, thetaBits))  return false;
			}
			else
			{
				inverseThetaBits = this.maxBits - thetaBits;
				if (!SetPositionBitsWait(this.id, thetaBits))  return false;
				if (!SetPositionBitsWait(this.id + 10, inverseThetaBits))  return false;
			}
			return ExecuteBroadcast();

		}
		/// <summary>
		/// Sets position of the desired servo in bites. Instantaneously executed.
		/// </summary>
		/// <param name="id">Dynamixel ID</param>
		/// <param name="thetaBits">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetPositionBits(int id, int thetaBits)
		{
			int inverseThetaBits;

			if (this.aligned)
			{
				if (!SetPositionBitsWait(id, thetaBits))  return false;
				if (!SetPositionBitsWait(id + 10, thetaBits)) return false;
			}
			else
			{
				inverseThetaBits = this.maxBits - thetaBits;
				if (!SetPositionBitsWait(id, thetaBits)) return false;
				if (!SetPositionBitsWait(id + 10, inverseThetaBits)) return false;
			}
			return ExecuteBroadcast();

		}
		/// <summary>
		///  Sets servo position. Instantaneously executed.
		/// </summary>
		/// <param name="theta">double Angle [radians]</param>
		/// <returns>bool</returns>
		public override bool SetPosition(double theta)
		{

			int thetaBits;

			if (this.cw)
			{
				//theta = (-(ToRadians(90 - this.ceroPosition) + theta));
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(90 + 60)) * (1023 / ToRadians(300))));
				
				theta = (this.maxDegrees - (this.ceroPosition + this.negativeRange)) + ToDegrees(theta);
				thetaBits = Convert.ToInt16(Math.Round((theta)) * (this.maxBits / (this.maxDegrees)));
			}
			else
			{
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + 60)) * (1023 / ToRadians(300))));
				thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
				
			}


			return SetPositionBits(thetaBits);

		}
		/// <summary>
		/// Sets buffer servo position in bites. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetPositionBitsWait(int thetaBits)
		{
			int inverseThetaBits;

			if (this.aligned)
			{
				if (!SetPositionBitsWait(this.id, thetaBits)) return false;
				if (!SetPositionBitsWait(this.id + 10, thetaBits)) return false;
			}
			else
			{
				inverseThetaBits = this.maxBits - thetaBits;
				if (!SetPositionBitsWait(this.id, thetaBits)) return false;
				if (!SetPositionBitsWait(this.id + 10, inverseThetaBits)) return false;
			}
			return true;
		}
		/// <summary>
		/// Sets buffer position of the desired Servo. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="theta">double Angle [radians]</param>
		/// <returns>bool</returns>
		public override bool SetPositionWait(double theta)
		{

			int thetaBits;

			if (this.cw)
			{
				//theta = (-(ToRadians(90 - this.ceroPosition) + theta));
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(90 + 60)) * (1023 / ToRadians(300))));
				theta = (this.maxDegrees - (this.ceroPosition + this.negativeRange)) + ToDegrees(theta);
				thetaBits = Convert.ToInt16(Math.Round((theta)) * (this.maxBits / (this.maxDegrees)));
			}
			else
			{
				//thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + 60)) * (1023 / ToRadians(300))));
				thetaBits = Convert.ToInt16(Math.Round((theta + ToRadians(this.ceroPosition + this.negativeRange)) * (this.maxBits / ToRadians(this.maxDegrees))));
			}


			return SetPositionBitsWait(thetaBits);

		}
		#endregion

		#region Speed
		/// <summary>
		/// Sets servo speed in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="omegaBits">int Speed [0,1023[bits]</param>
		/// <returns>bool</returns>
		public override bool SetSpeedBits(int omegaBits)
		{
		  
				if (!SetSpeedBitsWait(this.id, omegaBits)) return false;
				if (!SetSpeedBitsWait(this.id + 10, omegaBits)) return false;
		  
			return ExecuteBroadcast();

		}
		/// <summary>
		/// Sets speed of the desired Servo in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="omegaBits">int Speed [0,1023[bits]</param>
		/// <returns>bool</returns>
		public override bool SetSpeedBits(int id, int omegaBits)
		{

			if (!SetSpeedBitsWait(id, omegaBits)) return false;
			if (!SetSpeedBitsWait(id + 10, omegaBits)) return false;

			return ExecuteBroadcast();

		}
		/// <summary>
		/// Sets Sevro Speed.Instantaneously executed.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public override bool SetSpeedPerc(double omega)
		{
			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / 100.0, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBits(speed);

		}
		/// <summary>
		/// Sets Sevro Speed.Instantaneously executed.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public override bool SetSpeed(double omega)
		{
			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / this.maxServoSpeed, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBits(speed);

		}
		/// <summary>
		/// Sets buffer servo speed in bites. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="omegaBits">double Speed [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetSpeedBitsWait(int omegaBits)
		{
			if (!SetSpeedBitsWait(this.id, omegaBits)) return false;
			if (!SetSpeedBitsWait(this.id + 10, omegaBits)) return false;
			return true;
		}
		/// <summary>
		/// Sets buffer servo speed. Not Executed. Waits for Execute Instrucction.
		/// </summary>
		/// <param name="omega">Servo Speed [radians/second]</param>
		/// <returns>bool</returns>
		public override bool SetSpeedWaitPerc(double omega)
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
		public override bool SetSpeedWait(double omega)
		{
			int speed;

			omega = Math.Abs(omega);

			speed = Convert.ToInt16(Math.Round(omega * (1023.0) / this.maxServoSpeed, 0));		// Convierte velocidad a bits
			if (speed == 0) speed = 1;		// En la serie DX-64, si el valor de la velocidad es cero, motor gira libremente, sin control de velocidad. Por lo tanto la velocidad mínimo es 1.

			return SetSpeedBitsWait(speed);

		}
		#endregion

		#region Torque
		/// <summary>
		/// Sets servo torque in bits. Instantaneusly executed.
		/// </summary>
		/// <param name="direction">0 - CCW, 1 - CW</param>
		/// <param name="torqueBits">torque in bits [0,1023]</param>
		/// <returns>bool</returns>
		public override bool SetTorqueBits(bool direction, int torqueBits)
		{
            if (this.aligned)
            {
                if (!SetTorqueBits(this.id, direction, torqueBits)) return false;
                if (!SetTorqueBits(this.id + 10, direction, torqueBits)) return false;
            }
            else
            {
                bool codirection;
                if (direction == true) codirection = false;
                else codirection = true;
                if (!SetTorqueBits(this.id, direction, torqueBits)) return false;
                if (!SetTorqueBits(this.id + 10, codirection, torqueBits)) return false;
            }
            return true;

		} //
		/// <summary>
		/// Set Torque in percentage
		/// </summary>
		/// <param name="torquePercentaje">double Percentaje [0,100] %</param>
		/// <returns>bool</returns>
		public override bool SetTorquePercentaje(double torquePercentaje)
		{
			bool direction;
			int torqueBits;

			if (torquePercentaje <= 100 && torquePercentaje >= -100)
			{

				if (this.cw)
				{

					if (torquePercentaje >= 0)
					{
						direction = true;
						torquePercentaje = Math.Abs(torquePercentaje);
					}
					else
					{
						direction = false;
						torquePercentaje = Math.Abs(torquePercentaje);
					}

				}
				else
				{
					if (torquePercentaje >= 0)
					{
						direction = false;
						torquePercentaje = Math.Abs(torquePercentaje);
					}
					else
					{
						direction = true;
						torquePercentaje = Math.Abs(torquePercentaje);
					}
				}


			}
			else
			{
				direction = false;
				return false;
			}

			torqueBits = Convert.ToInt16((torquePercentaje * 1023.0) / 100.0);
			return SetTorqueBits(direction, torqueBits);


		}
		/// <summary>
		/// Set torque in Kgf.cm
		/// </summary>
		/// <param name="torque">double Torque [0,maxServoTorque][kgf.cm]</param>
		/// <returns>bool</returns>
		public override bool SetTorque(double torque)
		{
			bool direction;
			int torqueBits;

			if (torque <= 100 && torque >= -100)
			{

				if (this.cw)
				{

					if (torque >= 0)
					{
						direction = true;
						torque = Math.Abs(torque);
					}
					else
					{
						direction = false;
						torque = Math.Abs(torque);
					}

				}
				else
				{
					if (torque >= 0)
					{
						direction = false;
						torque = Math.Abs(torque);
					}
					else
					{
						direction = true;
						torque = Math.Abs(torque);
					}
				}


			}
			else
			{
				direction = false;
				return false;
			}

			torqueBits = Convert.ToInt16((torque * 1023.0) / this.MaxServoTorque);
			return SetTorqueBits(direction, torqueBits);


		}
		#endregion

		#region Servo Configuration

		/// <summary>
		/// Sets servo CW Limit in bites. Instantaneously executed.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetMaxCWBits(int bits)
		{
			int inverseBits;
			if (this.aligned)
			{
				if(!SetMaxCWBits(this.id, bits)) return false;
				if(!SetMaxCWBits(this.id + 10, bits)) return false;
			}
			else {
				if (bits != 0)
				{
					inverseBits = 1023 - bits;
					
				}
				else {
					inverseBits = 0;
				}
				if (!SetMaxCWBits(this.id, bits)) return false;
				if (!SetMaxCCWBits(this.id + 10, inverseBits)) return false;
			}
			return true;
		}
		/// <summary>
		/// Sets servo CW Limit. Reference is Servo Cero Position.
		/// </summary>
		/// <param name="angle">Angle [degrees]</param>
		/// <returns>bool</returns>
		public override bool SetMaxCW(double angle)
		{
			int thetaBits;
			if (angle >= mechanicalLimits[0])
			{
				thetaBits = Convert.ToInt16(Math.Round((angle + (this.ceroPosition + 60.0)) * (1023.0 / (300.0))));
				return SetMaxCWBits(thetaBits);
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + ": CW Limit is less than Mechanical Limit");
				return false;
			}
		}
		/// <summary>
		/// Sets servo CCW Limit in bites. Instantaneously executed.
		/// </summary>
		/// <param name="theta">int Angle [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetMaxCCWBits(int bits)
		{
			int inverseBits;
			if (this.aligned)
			{
				if (!SetMaxCCWBits(this.id, bits)) return false;
				if (!SetMaxCCWBits(this.id + 10, bits)) return false;
			}
			else
			{
				if (bits != 0)
				{
					inverseBits = 1023 - bits;

				}
				else
				{
					inverseBits = 0;
				}
				
				if (!SetMaxCCWBits(this.id, bits)) return false;
				if (!SetMaxCWBits(this.id+10, inverseBits)) return false;
				
			}
			return true;
		}
		/// <summary>
		/// Sets servo CCW Limit. Reference is Servo Cero Position.
		/// </summary>
		/// <param name="angle"></param>
		/// <returns></returns>
		public override bool SetMaxCCW(double angle)
		{

			int thetaBits;
			if (angle <= mechanicalLimits[1])
			{
				thetaBits = Convert.ToInt16(Math.Round((angle + (this.ceroPosition + 60.0)) * (1023.0 / (300.0))));
				return SetMaxCCWBits(thetaBits);
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + this.id + ": CCW Limit is greater than Mechanical Limit");
				return false;
			}

		}
		/// <summary>
		/// Enables Torque Servo Control
		/// </summary>
		/// <param name="enable">bool enable</param>
		/// <returns>bool</returns>
		public override bool SetTorqueControl(bool enable)
		{

			if (enable)
			{
				if (!SetMaxCWBits(0)) return false;
				if (!SetMaxCCWBits(0)) return false;
			}
			else
			{
				if (!SetMaxCWBits(1)) return false;
				if (!SetMaxCCWBits(1023)) return false;
			}

			return true;

		}
			

		/// <summary>
		/// Enables Servo Torque
		/// </summary>
		/// <param name="enable">0 - Disable Torque. 1 - Enable Torque</param>
		/// <returns>Bool.</returns>
		/*public override bool TorqueEnable(int enable)
		{
			if(!TorqueEnable(this.id, enable))return false;
			if(!TorqueEnable(this.id + 10, enable)) return false;
			return true;
		}*/
		/// <summary>
		/// Gets Torque Enabled State
		/// </summary>
		/// <param name="enabled">0 - Torque Disabled; 1 - Torque Enabled</param>
		/// <returns>bool</returns>
		public override bool GetTorqueEnabled(out int enabled)
		{
			int enabled1, enabled2;
			enabled = 2;
			if (!ReadData(this.id, 24, out enabled1)) return false; 
			if (!ReadData(this.id+10, 24, out enabled2)) return false;

			if (enabled1 == enabled2)
			{
				enabled = enabled1;
				return true;
			}
			else {
				enabled = 2;
				return false;
			}

			


		}

		/// <summary>
		/// Sets Baudrate
		/// </summary>
		/// <param name="baudrate">int BaudRate</param>
		/// <returns>bool</returns>
		public override bool SetBaudrate(double baudrate)
		{
			if(!SetBaudrate(this.id, baudrate)) return false;
			if (!SetBaudrate(this.id + 10, baudrate)) return false;
			return true;
		 }

		/// <summary>
		/// Writes data in a register
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data">int data</param>
		/// <returns>bool</returns>
		public override bool WriteData(int address, int data) {

			if (!WriteData(this.id, address, data)) return false;
			if (!WriteData(this.id + 10, address, data)) return false;
			return true;
		}
		/// <summary>
		/// Writes data un a register. Same address different data to every servo
		/// </summary>
		/// <param name="address"></param>
		/// <param name="data1"></param>
		/// <param name="data2"></param>
		/// <returns></returns>
		public bool WriteData(int address, int data1,int data2)
		{
			if (!WriteData(this.id, address, data1)) return false;
			if (!WriteData(this.id + 10, address, data2)) return false;
			return true;
		}
		/// <summary>
		/// Reads data from the register
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data">out int data</param>
		/// <returns>bool</returns>
		public override bool ReadData(int address, out int data)
		{
			int data1, data2;
			if (!ReadData(this.id, address, out data1)) {
				data = 0;
				return false; }
			if (!ReadData(this.id, address, out data2)) {
				data = 0;
				return false;
			}
			if (data1 == data2)
			{
				data = data1;
				return true;
			}
			else {
				data = 0;
				return false;
			}
			


		}
		/// <summary>
		/// Reads data form the same address of  Dual Servo
		/// </summary>
		/// <param name="address">int address</param>
		/// <param name="data1">out int data 1</param>
		/// <param name="data2">out int data 2</param>
		/// <returns></returns>
		public bool ReadData(int address, out int data1, out int data2)
		{
			
			if (!ReadData(this.id,address, out data1))
			{
				data1 = 0;
				data2 = 0;
				return false;
			}
			if (!ReadData(this.id + 10,address, out data2))
			{
				//data1 = 0;
				data2 = 0;
				return false;
			}
			return true;
		}

		/// <summary>
		/// Sets Maximum Torque to apply in bits
		/// </summary>
		/// <param name="torque">Max Torque [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetTorqueLimitBits(int torqueBits)
		{
			if(!SetTorqueLimitBits(this.id,torqueBits)) return false;
			if(!SetTorqueLimitBits(this.id + 10, torqueBits)) return false;
			return true;
		}
		/// <summary>
		/// Sets Maximum Torque to apply in percentage %
		/// </summary>
		/// <param name="torque">torque [0,100][%]</param>
		/// <returns>bool</returns>
		public override bool SetTorqueLimitPercentage(double torque)
		{
			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / 100);
			return SetTorqueLimitBits(torqueBits);
		}
		/// <summary>
		/// Sets Maximum Torque to apply in kgf.cm
		/// </summary>
		/// <param name="torque">torque [kgf.c]</param>
		/// <returns></returns>
		public override bool SetTorqueLimit(double torque)
		{

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / this.maxServoTorque);
			return SetTorqueLimitBits(torqueBits);

		}

		/// <summary>
		/// Sets Maximum Torque to apply in bits
		/// </summary>
		/// <param name="torque">Max Torque [0,1023][bits]</param>
		/// <returns>bool</returns>
		public override bool SetMaxTorqueBits(int torqueBits)
		{
			if (!SetMaxTorqueBits(this.id, torqueBits)) return false;
			if (!SetMaxTorqueBits(this.id + 10, torqueBits)) return false;
			return true;
		}
		/// <summary>
		/// Sets Maximum Torque to apply in percentage %
		/// </summary>
		/// <param name="torque">torque [0,100][%]</param>
		/// <returns>bool</returns>
		public override bool SetMaxTorquePercentage(double torque)
		{
			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / 100);
			return SetMaxTorqueBits(torqueBits);
		}
		/// <summary>
		/// Sets Maximum Torque to apply in kgf.cm
		/// </summary>
		/// <param name="torque">torque [kgf.c]</param>
		/// <returns></returns>
		public override bool SetMaxTorque(double torque)
		{

			int torqueBits;

			torqueBits = Convert.ToInt16((torque * 1023) / this.maxServoTorque);
			return SetMaxTorqueBits(torqueBits);

		}

		/// <summary>
		/// Sets Servo ID.
		/// </summary>
		/// <param name="id">ID desired</param>
		/// <returns>bool</returns>
		public override bool SetID(int id)
		{
			if (!SetID(this.id, id)) return false;
			if (!SetID(this.id + 10, id + 10)) return false;
			return true;
		}

		/// <summary>
		/// Chages the Status Return Level
		/// 0 - Do not respond to any instruction
		/// 1 - Respond only to Read_Data instruction
		/// 2 - Respond to all instructions
		/// </summary>
		/// <param name="l">int [0,2]</param>
		/// <returns>bool</returns>
		public override bool SetStatusReturnLevel(int l)
		{
			if (!SetStatusReturnLevel(this.id, l)) return false;
			if (!SetStatusReturnLevel(this.id + 10, l)) return false;
			return true;
		}


		#endregion

		#region Status

		/// <summary>
		/// Gets Load in bits
		/// </summary>
		/// <param name="loadBits">out int loadBits [0-2046] => [0-100%], addition of load1 + load2</param>
		/// <param name="sign">out int sign</param>
		/// <returns></returns>
		/*public override bool GetLoadBits(out int loadBits, out int sign)
		{
			int load1, load2;
			int sign1, sign2;
			sign = 0;
			if (!GetLoadBits(this.id, out load1,out sign1))
			{
				loadBits = 0;
				return false;
			}
			if (!GetLoadBits(this.id + 10, out load2, out sign2))
			{
				loadBits = 0;
				return false;
			}

			loadBits = (load1 + load2);
			
			if(sign1 != sign2) return false;

			sign = sign1;
			
			return true;

		}*/
		/// <summary>
        /// Gets Servo Load in Bits
		/// </summary>
		/// <param name="sign1">out int sign1 => direction of servo1</param>
		/// <param name="loadBits1">out int loadBits1 => [0-1023] </param>
        /// <param name="sign2">out int sign2 => direction of servo1</param>
        /// <param name="loadBits2">out int loadBits2 => [0-1023]</param>
		/// <returns></returns>
		public bool GetLoadBits(out int sign1, out int loadBits1, out int sign2, out int loadBits2)
		{
			loadBits1 = 0;
			loadBits2 = 0;
			sign1 = 0;
			sign2 = 0;

			if (!GetLoadBits(this.id, out loadBits1,out sign1))
			{
				return false;
			}
			if (!GetLoadBits(this.id + 10, out loadBits2, out sign2))
			{
			  return false;
			}

			return true;
			
		}
		/// <summary>
		/// Gets Servo Load in kgf.cm
		/// </summary>
		/// <param name="load">out double load [kgf.cm] referenced to maxServoTorque</param>
		/// <returns></returns>
		public override bool GetLoad(out double load)
		{
			int loadBits1, loadBits2;
			int sign1, sign2,sign;
			load = 0;
			if (!GetLoadBits(out sign1, out  loadBits1, out sign2, out loadBits2))
			{
				load = 0;
				return false;
			}

            if (this.aligned)
            {
                if (sign1 == sign2) { sign = sign1; }
                else { return false; }
                load = sign * ((loadBits1 + loadBits2) * this.maxServoTorque) / 2046.0;
            }
            else
                load = (((loadBits1 + loadBits2) / 2) * this.maxServoTorque) / 1023.0;
			return true;

		}
		/// <summary>
        /// Gets Servo Loads Separately 
		/// </summary>
		/// <param name="load1">out double load1 [0,maxServoTorque/2][kgf.cm]</param>
        /// <param name="load2">out double load2 [0,maxServoTorque/2][kgf.cm]</param>
		/// <returns></returns>
		public bool GetLoad(out double load1, out double load2)
		{
			int loadBits1, loadBits2;
			int sign1, sign2;
			load1 = 0;
			load2 = 0;

			if (!GetLoadBits(out loadBits1, out sign1, out loadBits2, out sign2)) return false;

			load1 = sign1 * (loadBits1 * (this.maxServoTorque / 2.0) / 1023.0);
			load2 = sign2 * (loadBits2 * (this.maxServoTorque / 2.0) / 1023.0);
			return true;

		}
		/// <summary>
		/// Gets Servo Load in percentage (with sign)
		/// </summary>
		/// <param name="load">out double load [0-100%]</param>
		/// <returns></returns>
		public override bool GetLoadPercentage(out double load)
		{
			int loadBits1, loadBits2;
			int sign1, sign2, sign;
			load = 0;
			if (!GetLoadBits(out sign1, out loadBits1, out sign2, out loadBits2))
			{
				load = 0;
				return false;
			}

            if (this.aligned)
            {
                if (sign1 == sign2) { sign = sign1; }
                else { return false; }
                load = sign * ((loadBits1 + loadBits2) * 100.0) / 2046.0;
            }
            else
                load =(((loadBits1 + loadBits2) / 2) * 100.0) / 1023.0;
			return true;

		}
		/// <summary>
		/// Gets Servo Load in percentage separately
		/// </summary>
		/// <param name="load1">out double load1 [0-100%]</param>
        /// <param name="load2">out double load2 [0-100%]</param>
		/// <returns></returns>
		public bool GetLoadPercentage(out double load1, out double load2)
		{
			int loadBits1, loadBits2;
			int sign1, sign2;
			load1 = 0;
			load2 = 0;

			if (!GetLoadBits(out loadBits1, out sign1, out loadBits2, out sign2)) return false;

			load1 = sign1 * (loadBits1 * (100.0) / 1023.0);
			load2 = sign2 * (loadBits2 * (100.0) / 1023.0);
			return true;

		}

		/// <summary>
		/// Gets Programmed servo speed
		/// </summary>
		/// <param name="data">out double Speed [radians/second]</param>
		/// <returns>bool</returns>
		public override bool GetTorqueLimitBits(out int torqueLimitBits)
		{
			int torqueLimitBits1, torqueLimitBits2;
			if (!GetTorqueLimitBits(this.id,out torqueLimitBits1))
			{
				torqueLimitBits = 0;
				return false;
			}
			if (!GetTorqueLimitBits(this.id+10, out torqueLimitBits2))
			{
				torqueLimitBits = 0;
				return false;
			}

			if (torqueLimitBits1 == torqueLimitBits2)
			{
				torqueLimitBits = torqueLimitBits1;
				return true;
			}
			else
			{
				torqueLimitBits = 0;
				return false;
			}


		}
		/// <summary>
		/// Gets Programmed servo speed
		/// </summary>
		/// <param name="data">out double Speed [radians/second]</param>
		/// <returns>bool</returns>
		public bool GetTorqueLimitBits(out int torqueLimitBits1, out int torqueLimitBits2)
		{
		
			if (!GetTorqueLimitBits(this.id, out torqueLimitBits1))
			{
				torqueLimitBits1 = 0;
				torqueLimitBits2 = 0;
				return false;
			}
			if (!GetTorqueLimitBits(this.id + 10, out torqueLimitBits2))
			{
				torqueLimitBits2 = 0;
				return false;
			}

			return true;



		}
		/// <summary>
		/// Gets Programmed servo speed
		/// </summary>
		/// <param name="data">out double Speed [radians/second]</param>
		/// <returns>bool</returns>
		public override bool GetTorqueLimit(out double torqueLimit)
		{
			int torqueLimitBits1, torqueLimitsBits2;

			if (!GetTorqueLimitBits(out torqueLimitBits1, out torqueLimitsBits2))
			{
				torqueLimit = 0;
				return false;
			}

			torqueLimit = ((torqueLimitBits1 + torqueLimitsBits2) * this.maxServoTorque) / 2046.0;
			return true;

		}
		/// <summary>
		/// Gets Programmed servo speed
		/// </summary>
		/// <param name="data">out double Speed [radians/second]</param>
		/// <returns>bool</returns>
		public bool GetTorqueLimit(out double torqueLimit1, out double torqueLimit2)
		{
			int torqueLimitBits1, torqueLimitBits2;

			if (!GetTorqueLimitBits(out torqueLimitBits1, out torqueLimitBits2))
			{
				torqueLimit1 = 0;
				torqueLimit2 = 0;
				return false;
			}

			torqueLimit1 = ((torqueLimitBits1) * this.maxServoTorque / 2.0) / 1023.0;
			torqueLimit2 = ((torqueLimitBits2) * this.maxServoTorque / 2.0) / 1023.0;
			return true;

		}
		/// <summary>
		/// Gets Programmed servo speed
		/// </summary>
		/// <param name="data">out double Speed [radians/second]</param>
		/// <returns>bool</returns>
		public override bool GetTorqueLimitPercentage(out double torqueLimit)
		{
			int torqueLimitBits1, torqueLimitsBits2;

			if (!GetTorqueLimitBits(out torqueLimitBits1, out torqueLimitsBits2))
			{
				torqueLimit = 0;
				return false;
			}
			if (torqueLimitBits1 == torqueLimitsBits2)
			{
				torqueLimit = ((torqueLimitBits1) * 100.0) / 1023.0;
				return true;
			}
			else {
				torqueLimit = 0;
				return false;
			}

		}

		

		/// <summary>
		/// Gets average Voltage of Dual Servos
		/// </summary>
		/// <param name="voltage">Average Voltage</param>
		/// <returns>bool</returns>
		public override bool GetVoltage(out double voltage)
		{
			double voltage1, voltage2;
		 
					if (!GetVoltage(this.id, out voltage1))
					{
						voltage = 0;
						return false;
					}
							 
					if (!GetVoltage(this.id + 10, out voltage2))
					{
						voltage = 0;
						return false;
					}
				  
			   
					voltage = ( voltage1 + voltage2 ) / 2.0;
					return true;

					

			}
		/// <summary>
		/// Gets both voltages of Dual Servo
		/// </summary>
		/// <param name="voltage1">Voltage of the 1st Servo</param>
		/// <param name="voltage2">Voltage of the 2dn Servo</param>
		/// <returns></returns>
		public bool GetVoltage(out double voltage1, out double voltage2)
		{
			
			if (!GetVoltage(this.id, out voltage1))
			{
				voltage1 = 0;
				voltage2 = 0;
				return false;
			}

			if (!GetVoltage(this.id + 10, out voltage2))
			{
				voltage2 = 0;
				return false;
			}

			return true;

		}

		/// <summary>
		/// Gets both temperatures of Dual Servos
		/// </summary>
		/// <param name="temp1">Temperature of 1st Servo [°C]</param>
		/// <param name="temp2">Temperature of 2nd Servo [°C]</param>
		/// <returns>bool</returns>
		public virtual bool GetTemp(out double temp1, out double temp2)
		{

			if (!GetTemp(this.id, out temp1))
			{
				temp1 = 0;
				temp2 = 0;
				return false;
			}

			if (!GetTemp(this.id + 10, out temp2))
			{
				temp2 = 0;
				return false;
			}

			return true;

		}
		/// <summary>
		/// Gets both temperatures of Dual Servos
		/// </summary>
		/// <param name="temp1">Temperature of 1st Servo [°C]</param>
		/// <param name="temp2">Temperature of 2nd Servo [°C]</param>
		/// <returns>bool</returns>
		public override bool GetTemp(out double temp)
		{
			double temp1, temp2;
			if (!GetTemp(this.id, out temp1))
			{
				temp = 0;
				return false;
			}

			if (!GetTemp(this.id + 10, out temp2))
			{
				temp = 0;
				return false;
			}
			temp = (temp1 + temp2) / 2.0;

			return true;

		}


		/// <summary>
		/// Show sings of life ja
		/// </summary>
		/// <returns>bool</returns>
		public bool Ping(out bool[] errors1,out bool[] errors2 )
		{
			errors1 = new bool[7];
			errors2 = new bool[7];

			if (!Ping(this.id,out errors1)) return false;
			if (!Ping(this.id+10, out errors2)) return false;

			return true;

			//if (!manipulatorStruct.serialPortManager.SendCommand(Cmd, "Ping", this.serialPort)) return false;
			//return true;

		}

		#endregion

		#region StatusInternal
		/// <summary>
		/// Sends Ping and get all errors in errors bool array internal variable.
		/// </summary>
		/// <returns>bool</returns>
		public override bool Ping()
		{
			if (!Ping(out errors1,out errors2)) return false;
			return true;
		}
		/// <summary>
		/// Gets Servo Torque Enable State in torqueEnabled internal variable
		/// </summary>
		/// <returns>bool</returns>
		public override bool GetTorqueEnabled()
		{
			int e = 0;
			if (!GetTorqueEnabled(out e)) return false;
			if (e == 0) torqueEnabled = false;
			if (e == 1) torqueEnabled = true;
			return true;

		}
		/// <summary>
		/// Gets Servo Torque Limit in Percentage in torqueLimitPerc internal variable
		/// </summary>
		/// <returns>bool</returns>
		public override bool GetTorqueLimitPercentage()
		{
			int torqueLimitBits1, torqueLimitsBits2;

			if (!GetTorqueLimitBits(out torqueLimitBits1, out torqueLimitsBits2))
			{
				torqueLimitPerc = 0;
				return false;
			}
			if (torqueLimitBits1 == torqueLimitsBits2)
			{
				torqueLimitPerc = ((torqueLimitBits1) * 100.0) / 1023.0;
				return true;
			}
			else
			{
				torqueLimitPerc = 0;
				return false;
			}

		}
		/// <summary>
		/// Gets Servo Current Load in a load variable
		/// </summary>
		/// <returns></returns>
		public override bool GetLoad()
		{
			if (!GetLoadPercentage(out load)) return false;
			return true;
		}
		/// <summary>
		/// Gets Servo Temperature in temp variable
		/// </summary>
		/// <returns></returns>
		public override bool GetTemp()
		{
			if (!GetTemp(out temp)) return false;
			return true;
		}
		/// <summary>
		/// Gets Servo Voltage in voltage variable
		/// </summary>
		/// <returns></returns>
		public override bool GetVoltage()
		{
			double v;
			if (!GetVoltage(out v)) return false;
			Voltage = v;
			return true;
		}
		/// <summary>
		/// Executes Ping, GetTorqueEnabled,GetTorqueLimitPErcentage,GetTemp,GetVoltage,GetLoad
		/// </summary>
		/// <returns>bool</returns>
		public override bool GlobalStatus() {

			if (!Ping()) return false;
			if (!GetTorqueEnabled()) return false;
			if (!GetTorqueLimitPercentage()) return false;
			if (!GetTemp()) return false;
			if (!GetLoad()) return false;
			if (!GetVoltage()) return false;
			return true;
		}
		#endregion

		#endregion

	}
}
