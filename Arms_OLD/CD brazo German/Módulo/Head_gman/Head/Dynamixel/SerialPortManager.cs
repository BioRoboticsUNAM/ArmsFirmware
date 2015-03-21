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

		public event ByteArrayEventHandler byteError;

		public SerialPortManager()
		{
			SerialPort serialPortRS485;
			SerialPort serialPortTTL;
			
			serialPortRS485 = new SerialPort("COM8", 250000, Parity.None, 8, StopBits.One);
			serialPortRS485.ReadTimeout = 100;
			//serialPortRS485.ReceivedBytesThreshold = 6;
			dataAvaibleRS485 = false;
			bufferSizeRS485 = 6;

			//serialPortTTL = new SerialPort("COM5", 250000, Parity.None, 8, StopBits.One);
			//serialPortTTL.ReadTimeout = 100;
			////serialPortTTL.ReceivedBytesThreshold = 6;
			//dataAvaibleTTL = false;
			//bufferSizeTTL = 6;

			serialPorts = new SortedList<string, SerialPort>();
			registeredSerialPorts = new SortedList<ServoType, SerialPort>();

			//serialPorts.Add("COM12", serialPortRS485);
			//serialPorts.Add("COM11", serialPortTTL);

			//AddSerialPort(ServoType.EX106, serialPortRS485);
			AddSerialPort(ServoType.RX64, serialPortRS485);
			//AddSerialPort(ServoType.AX12, serialPortTTL);

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
				catch { TextBoxStreamWriter.DefaultLog.WriteLine("Cannot Open Serial Port " + sp.PortName); }
			
		}
		public void ClosePorts() {
			foreach (SerialPort sp in serialPorts.Values) try { if (sp.IsOpen) sp.Close(); }
				catch { }
		}

		public bool SendCommand(ServoType servoType, byte[] cmd) 
		{
			SerialPort sp = registeredSerialPorts[servoType];
			if (sp == null) return false;
			EraseBuffer(servoType);
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
				//TextBoxStreamWriter.DefaultLog.WriteLine(" Serial Port Manager => Serial Transmission Problem");
				System.Console.WriteLine(" Serial Port Manager => Serial Transmission Problem");
				//byte[] byteErr = null;
				//byteError(cmd);
				
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
			//while (sp.BytesToRead == 0) {
			//    Thread.Sleep(0);
			//    sp = registeredSerialPorts[servoType];
			//}
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
					TextBoxStreamWriter.DefaultLog.WriteLine(" Serial Port Manager => Serial Reception Problem");
					byte[] byteErr = null;
					//byteError(byteErr);
					return false;
				}

			}
			if (inCmd[4] != 0) {
				//byteError(inCmd);
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
				//TextBoxStreamWriter.DefaultLog.WriteLine("Serial Port Manager => Erase Buffer: Serial Port is not Ready");
				System.Console.WriteLine("Serial Port Manager => Erase Buffer: Serial Port is not Ready");
				byte[] byteErr = null;
				//byteError(byteErr);
				return false; }
			return true;
		}
		///// <summary>
		///// Sends any kind of command.
		///// </summary>
		///// <param name="cmd">byte[] Instruction Packet</param>
		///// <param name="nameCmd">Command Name for Error reporting</param>
		///// <returns></returns>
		//private bool SendCommandRS485(byte[] cmd, string cmdName)
		//{
		//    string id = cmd[2].ToString();

		//    if (!serialPortRS485.IsOpen)
		//    {
		//        TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + ": Serial RS485 Port Closed");
		//        return false;
		//    }
		//    else
		//    {
		//        serialPortRS485.Write(cmd, 0, cmd.Length);
		//        if (reportExecuted) TextBoxStreamWriter.DefaultLog.WriteLine(cmdName + ": Executed");
				
		//        return true;
		//    }


		//}
		//private bool SendAndReceiveCommandRS485(byte[] cmd, string cmdName, out byte[] inCmd, int buffer) {

		//    string id = cmd[2].ToString();
		//    inCmd = null;

		//    serialPortRS485.ReceivedBytesThreshold = buffer;
		//    bufferSizeRS485 = buffer;

		//    if (!SendCommandRS485(cmd, cmdName)) return false;


		//   if (!ReceiveCommandSyncRS485(out inCmd)) return false;

		//    //CommandReceived(inCmd);
		//   serialPortRS485.ReceivedBytesThreshold = 6;
		//    return true;
		
		
		//}
		//private bool ReceiveCommandSyncRS485(out byte[] inCmd)
		//{

			
		//    int count = 0;
		//    inCmd = null;


		//    while (!dataAvaibleRS485)
		//    {
		//        Thread.Sleep(0);
		//        count++;
		//        if (count > 20000) return false;

		//    }

		//    dataAvaibleRS485 = false;

		//    inCmd = queueArrayRS485;
			
		//    return true;
		//}


		//private bool SendCommandTTL(byte[] cmd, string cmdName)
		//{
		//    string id = cmd[2].ToString();

		//    if (!serialPortTTL.IsOpen)
		//    {
		//        TextBoxStreamWriter.DefaultLog.WriteLine("Servo " + id + ": Serial TTL Port Closed");
		//        return false;
		//    }
		//    else
		//    {
		//        serialPortTTL.Write(cmd, 0, cmd.Length);

		//        if (reportExecuted) TextBoxStreamWriter.DefaultLog.WriteLine(cmdName + ": Executed");
				
		//        return true;
		//    }


		//}
		//private bool SendAndReceiveCommandTTL(byte[] cmd, string cmdName, out byte[] inCmd, int buffer)
		//{

		//    string id = cmd[2].ToString();
		//    inCmd = null;

		//    serialPortTTL.ReceivedBytesThreshold = buffer;
		//    bufferSizeTTL = buffer;

		//    if (!SendCommandTTL(cmd, cmdName)) return false;


		//    if (!ReceiveCommandSyncTTL(out inCmd)) return false;

		//    //CommandReceived(inCmd);
		//    serialPortTTL.ReceivedBytesThreshold = 6;
		//    return true;


		//}
		//private bool ReceiveCommandSyncTTL(out byte[] inCmd)
		//{


		//    int count = 0;
		//    inCmd = null;


		//    while (!dataAvaibleTTL)
		//    {
		//        Thread.Sleep(0);
		//        count++;
		//        if (count > 20000) return false;

		//    }
		//    dataAvaibleTTL = false;

		//    inCmd = queueArrayTTL;

		//    return true;
		//}

		//public bool SendCommand(byte[] cmd, string cmdName, SerialPort serialPort)
		//{

		//    if (serialPort == serialPortRS485)
		//    {
		//        return SendCommandRS485(cmd, cmdName);
		//    }

		//    if (serialPort == serialPortTTL)
		//    {
		//        return	SendCommandTTL(cmd, cmdName);
		//    }
		//    return false;
		//}
		//private bool ReceiveCommand(out byte[] inCmd, SerialPort serialPort) {

		//    if (serialPort == serialPortRS485) {
		//        return ReceiveCommandSyncRS485(out inCmd);
		//    }
		//    if (serialPort == serialPortTTL) {
		//        return ReceiveCommandSyncTTL(out inCmd);
		//    }
		//    inCmd = new byte[1];
		//    return false;
		//}
		//public bool SendAndReceiveCommand(byte[] cmd, string cmdName, out byte[] inCmd, int buffer, SerialPort serialPort) {

		//    if (serialPort == serialPortRS485) {
		//        return SendAndReceiveCommandRS485(cmd, cmdName, out inCmd, buffer);
		//    }

		//    if (serialPort == serialPortTTL) {
		//        return SendAndReceiveCommandTTL(cmd, cmdName, out inCmd, buffer);
		//    }
		//    inCmd = new byte[1];
		//    return false;

		//}

		


		///// <summary>
		///// Calculates the CheckSum for any Method
		///// </summary>
		///// <param name="Cmd">Bytes array of N-1 (N := number of arguments of the instruction)</param>
		///// <returns>Byte N (CheckSum</returns>
		///// 
		//protected byte CheckSum(byte[] Cmd)
		//{
		//    byte sum = 0;
		//    byte checkSum;
		//    unchecked
		//    {
		//        for (int i = 2; i < (Cmd.Length - 1); i++)
		//        {
		//            sum = (byte)(sum + Cmd[i]);
		//        }
		//    }
		//    checkSum = (byte)(255 - sum);

		//    return checkSum;
		//}
		
	

	}
}
