using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Threading;
using System.IO;
using Robotics.Controls;

namespace Head
{
	public delegate void ByteArrayEventHandler(byte[] s);
	
	public class SerialPortManager
	{
		
		public int bufferSizeRS485, bufferSizeTTL;
		
		public bool dataAvaibleRS485, dataAvaibleTTL;
		public bool reportExecuted;


		private SortedList<ServoType, SerialPort> registeredSerialPorts;
		public SortedList<string, SerialPort> serialPorts;

        public SerialPortManager(Settings set)
		{
			SerialPort serialPortRS485;
			
			
			serialPortRS485 = new SerialPort(set.com, set.baud, Parity.None, 8, StopBits.One);
			serialPortRS485.ReadTimeout = 100;
			
			dataAvaibleRS485 = false;
			bufferSizeRS485 = 6;

			
			serialPorts = new SortedList<string, SerialPort>();
			registeredSerialPorts = new SortedList<ServoType, SerialPort>();

			AddSerialPort(ServoType.RX64, serialPortRS485);
			
			reportExecuted = false;

		}

		public void AddSerialPort(ServoType servoType,string portCOM, int baudrate, Parity parity, int bits, StopBits stopBits) {

			if (!serialPorts.ContainsKey(portCOM))
			{
				SerialPort serialPort = new SerialPort(portCOM, baudrate, parity, bits, stopBits);
				serialPorts.Add(portCOM, serialPort);
			}
			if (!registeredSerialPorts.ContainsKey(servoType))
			{
				registeredSerialPorts.Add(servoType, serialPorts[portCOM]);
			}
		}
		public void AddSerialPort(ServoType servoType, SerialPort serialPort) {

			if (!serialPorts.ContainsKey(serialPort.PortName))
			{
				serialPorts.Add(serialPort.PortName, serialPort);
			}
			if (!registeredSerialPorts.ContainsKey(servoType))
			{
				registeredSerialPorts.Add(servoType, serialPort);
			}
	
		}

		public void OpenPorts() {

			foreach (SerialPort sp in serialPorts.Values) try { if (!sp.IsOpen) sp.Open(); }
				catch { TextBoxStreamWriter.DefaultLog.WriteLine("SerialPortManager: Cannot Open Serial Port " + sp.PortName); }
			
		}
		public void ClosePorts() {
			foreach (SerialPort sp in serialPorts.Values) try { if (sp.IsOpen) sp.Close(); }
				catch { }
		}

		public bool SendCommand(ServoType servoType, byte[] cmd) 
		{
			SerialPort sp = registeredSerialPorts[servoType];
			if (sp == null) return false;
			//EraseBuffer(servoType);
			try
			{
				if (sp.IsOpen)
				{
					sp.Write(cmd, 0, cmd.Length);
				}
				else
				{
					sp.Open();
					sp.Write(cmd, 0, cmd.Length);
				}
			}
			catch
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Serial Port Manager: Serial Transmission Problem");
				return false;
			}
			return true;
		}
		public bool SendReceiveCommand(ServoType servoType, byte[] cmd, int dataLength, out byte[] inCmd) {
			inCmd = null;
			if (!EraseBuffer(servoType)) return false;
			if (!SendCommand(servoType, cmd)) return false;
			if (!ReceiveCommand(servoType, dataLength, out inCmd)) return false;
			return true;
		}
		public bool ReceiveCommand(ServoType servoType, int receivedDataLength, out byte[] inCmd)
		{
			int counter = 0;
			int bufferLength = receivedDataLength + 6;
			inCmd = new byte[bufferLength];
			
			SerialPort sp = registeredSerialPorts[servoType];
			
			if (sp == null) return false;


			for (int i = 0; i < bufferLength; i++)
			{

				try
				{
					while (sp.BytesToRead == 0)
					{
						Thread.Sleep(0);
						counter++;
						if (counter > 50000) {
							return false;
						}
					}
					inCmd[i] = Convert.ToByte(sp.ReadByte());

					if (i == 0) {
						if (inCmd[0] != 255) {
							i--;
						}
					}

				}
				catch
				{
					TextBoxStreamWriter.DefaultLog.WriteLine("Serial Port Manager: Serial Reception Problem");
                    return false;
				}

			}
			if (inCmd[4] != 0) {
				
				return false;
			}
			return true;

		}

		private bool EraseBuffer(ServoType servoType) { 
		
			SerialPort sp = registeredSerialPorts[servoType];
			if (sp == null) return false;
			try
			{
				sp.DiscardInBuffer();
			}
			catch 
			{
				TextBoxStreamWriter.DefaultLog.WriteLine("Serial Port Manager: Erase Buffer error [Serial Port is not Ready]");
				
				return false; 
            }
			return true;
		}
	

	}
}
