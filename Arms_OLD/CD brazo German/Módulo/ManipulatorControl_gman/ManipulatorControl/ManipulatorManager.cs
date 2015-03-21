using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO.Ports;
using Robotics.API;
using Robotics.Controls;
using Robotics.API.PrimitiveSharedVariables;

namespace ManipulatorControl
{
    public delegate void ArmControlStatusChangedEH(ArmControlStatus status);
    public delegate void ThreeDoubleParser(double d1, double d2, double d3);
    public delegate void SixDoubleParser(double d1, double d2, double d3, double d4, double d5, double d6);
    public delegate void SevenDoubleParser(double d1, double d2, double d3, double d4, double d5, double d6, double d7);

    public class ManipulatorManager
    {
        private Manipulator leftArm;
        private Manipulator rightArm;
        private SortedList<string, MapObstacle> mapObstacles;
        private SortedList<string, MapNode> mapNodes;
        private SortedList<string, PredefPosition> leftPredefPos;
        private SortedList<string, PredefMovement> leftPredefMovs;
        private SortedList<string, PredefPosition> rightPredefPos;
        private SortedList<string, PredefMovement> rightPredefMovs;
        private MapOptimalPath optLeftPath;
        private MapOptimalPath optRightPath;
        private MapGoalPoint leftGoal;
        private MapGoalPoint rightGoal;

        private string fileLeftPredefPos = "LeftPositions.xml";
        private string fileLeftMovements = "LeftMovements.xml";
        private string fileRightPredefPos = "RightPositions.xml";
        private string fileRightMovements = "RightMovements.xml";

        public TaskPlanner taskPlanner;
        Thread checkSystemThread;
        public ArmControlStatus status;
        CommandManager cmdMan;
        ConnectionManager cnnMan;

        SerialPort leftArmSerialPort;
        SerialPort rightArmSerialPort;

        private IntSharedVariable lowBat;
        private DoubleArraySharedVariable sharedVarLeftArmPos;
        private DoubleArraySharedVariable sharedVarRightArmPos;

		private CmdArmsGoTo cmdArmsGoTo;
		private CmdArmsMove cmdArmsMove;
		private CmdArmsTorque cmdArmsTorque;
        private CmdArmsState cmdArmsState;
        private CmdGetVoltage cmdGetVoltage;

        private CmdRaState cmdRaState;
        private CmdRaAbsPos cmdRaAbsPos;
        private CmdRaArtPos cmdRaArtPos;
        private CmdRaGoTo cmdRaGoTo;
        private CmdRaMove cmdRaMove;
        private CmdRaOpenGripper cmdRaOpenGripper;
        private CmdRaCloseGrip cmdRaCloseGripper;
        private CmdRaReachable cmdRaReachable;
        private CmdRaRelPos cmdRaRelPos;
        private CmdRaTorque cmdRaTorque;

        private CmdLaState cmdLaState;
        private CmdLaAbsPos cmdLaAbsPos;
        private CmdLaArtPos cmdLaArtPos;
        private CmdLaGoTo cmdLaGoTo;
        private CmdLaMove cmdLaMove;
        private CmdLaOpenGripper cmdLaOpenGripper;
        private CmdLaCloseGrip cmdLaCloseGripper;
        private CmdLaReachable cmdLaReachable;
        private CmdLaRelPos cmdLaRelPos;
        private CmdLaTorque cmdLaTorque;

        public ManipulatorManager(string leftPortName, string rightPortName)
        {
            this.status = new ArmControlStatus();
            if (string.IsNullOrEmpty(leftPortName)) leftPortName = "COM5";
            if (string.IsNullOrEmpty(rightPortName)) rightPortName = "COM6";
            this.leftArmSerialPort = new SerialPort(leftPortName, 57600, Parity.None, 8, StopBits.One);
            this.rightArmSerialPort = new SerialPort(rightPortName, 57600, Parity.None, 8, StopBits.One);
            this.status.LeftComPort = leftPortName;
            this.status.RightComPort = rightPortName;
            this.leftArm = new Manipulator(this.leftArmSerialPort, ArmType.LeftArm);
            this.rightArm = new Manipulator(this.rightArmSerialPort, ArmType.RightArm);
            this.leftArm.ArmPositionChanged += new ArmPositionChangedEH(leftArm_ArmPositionChanged);
            this.rightArm.ArmPositionChanged += new ArmPositionChangedEH(rightArm_ArmPositionChanged);

            this.mapObstacles = new SortedList<string, MapObstacle>();
            this.mapNodes = new SortedList<string, MapNode>();
            this.leftPredefPos = new SortedList<string, PredefPosition>();
            this.leftPredefMovs = new SortedList<string, PredefMovement>();
            this.rightPredefPos = new SortedList<string, PredefPosition>();
            this.rightPredefMovs = new SortedList<string, PredefMovement>();
            this.optLeftPath = new MapOptimalPath();
            this.optRightPath = new MapOptimalPath();
            this.leftGoal = new MapGoalPoint();
            this.rightGoal = new MapGoalPoint();

            this.lowBat = new IntSharedVariable("batAlert");
            this.sharedVarLeftArmPos = new DoubleArraySharedVariable("leftArmPos");
            this.sharedVarRightArmPos = new DoubleArraySharedVariable("rightArmPos");

            this.cmdMan = new CommandManager();
            this.cnnMan = new ConnectionManager("ARMS", 2080, this.cmdMan);
            this.cnnMan.ClientConnected += new System.Net.Sockets.TcpClientConnectedEventHandler(cnnMan_ClientConnected);
            this.cnnMan.ClientDisconnected += new System.Net.Sockets.TcpClientDisconnectedEventHandler(cnnMan_ClientDisconnected);
			this.cmdMan.SharedVariablesLoaded += new SharedVariablesLoadedEventHandler(cmdMan_SharedVariablesLoaded);

            this.status.Running = true;
            this.checkSystemThread = new Thread(new ThreadStart(this.CheckSystemThreadTask));
            this.checkSystemThread.IsBackground = true;
            this.checkSystemThread.Start();
        }

		void cmdMan_SharedVariablesLoaded(CommandManager cmdMan)
		{
			if (!this.cmdMan.SharedVariables.Contains(this.lowBat.Name))
				this.cmdMan.SharedVariables.Add(this.lowBat);
			else
				this.lowBat = (IntSharedVariable)this.cmdMan.SharedVariables[this.lowBat.Name];
		}

        #region Cnn to BB methods

        private void cnnMan_ClientConnected(System.Net.Sockets.Socket s)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Client Connected");

            int temp = 0;
			string temps;
			temp = this.cmdMan.SharedVariables.LoadFromBlackboard(5000, out temps);
            if (!String.IsNullOrEmpty(temps))
                TextBoxStreamWriter.DefaultLog.WriteLine("Shared Vars Loading Error: " + temps);

            this.status.IsConnectedToBB = true;
            this.status.CnnToBBPort = 2080;
            this.status.AreVarsFromBBLoaded = true;
            this.OnStatusChanged(this.status);
        }

        private void cnnMan_ClientDisconnected(System.Net.EndPoint ep)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Client Connected");
            this.status.IsConnectedToBB = false;
            this.OnStatusChanged(this.status);
        }

        private void CreateLowBatSharedVar()
        {
            if (!this.status.IsLowBatCreated)
            {
                try
                {
                    if (!this.cmdMan.SharedVariables.Contains(this.lowBat.Name))
                        this.cmdMan.SharedVariables.Add(this.lowBat);
                    else this.lowBat = (IntSharedVariable)this.cmdMan.SharedVariables[this.lowBat.Name];
                    this.status.IsLowBatCreated = true;
                    TextBoxStreamWriter.DefaultLog.WriteLine("Shared variable " + this.lowBat.Name + " created");
                    this.OnStatusChanged(this.status);
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't create " +
                        this.lowBat.Name + " shared variable");
                }
            }
        }

        private void CreateLeftArmPosSharedVar()
        {
            if (!this.status.IsSharedVarLeftArmPosCreated)
            {
                try
                {
                    if (!this.cmdMan.SharedVariables.Contains(this.sharedVarLeftArmPos.Name))
                        this.cmdMan.SharedVariables.Add(this.sharedVarLeftArmPos);
                    else this.sharedVarLeftArmPos = (DoubleArraySharedVariable)this.cmdMan.SharedVariables[this.sharedVarLeftArmPos.Name];
                    this.status.IsSharedVarLeftArmPosCreated = true;
                    TextBoxStreamWriter.DefaultLog.WriteLine("Shared variable " + this.sharedVarLeftArmPos.Name + " created");
                    this.OnStatusChanged(this.status);
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't create " +
                        this.sharedVarLeftArmPos.Name + " shared variable");
                }
            }
        }

        private void CreateRightArmPosSharedVar()
        {
            if (!this.status.IsSharedVarRightArmPosCreated)
            {
                try
                {
                    if (!this.cmdMan.SharedVariables.Contains(this.sharedVarRightArmPos.Name))
                        this.cmdMan.SharedVariables.Add(this.sharedVarRightArmPos);
                    else this.sharedVarRightArmPos = (DoubleArraySharedVariable)this.cmdMan.SharedVariables[this.sharedVarRightArmPos.Name];
                    this.status.IsSharedVarRightArmPosCreated = true;
                    TextBoxStreamWriter.DefaultLog.WriteLine("Shared variable " + this.sharedVarRightArmPos.Name + " created");
                    this.OnStatusChanged(this.status);
                }
                catch
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't create " +
                        this.sharedVarRightArmPos.Name + " shared variable");
                }
            }
        }

        public void UpdateLowBatSharedVar(int low)
        {
			if (this.lowBat.Initialized)
			{
				if (this.status.IsLowBatCreated)
				{
					if (!this.lowBat.TryWrite(low))
						TextBoxStreamWriter.DefaultLog.WriteLine("Can't write " + this.lowBat.Name);
					else TextBoxStreamWriter.DefaultLog.WriteLine(lowBat.Name + " shared variable succesfully written");
				}
				else this.CreateLowBatSharedVar();
			}
			else
			{
				TextBoxStreamWriter.DefaultLog.WriteLine(lowBat.Name + " shared variable not initilized");
				this.lowBat.Initialize();
			}
        }
        
        private void UpdateLeftArmPosSharedVar()
        {
            if (this.status.IsSharedVarLeftArmPosCreated)
            {
                if (!this.sharedVarLeftArmPos.TryWrite(this.leftArm.CartesianPosition.v))
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can'r write " + this.sharedVarLeftArmPos.Name);
            }
            else this.CreateLeftArmPosSharedVar();
        }

        private void UpdaterightArmPosSharedVar()
        {
            if (this.status.IsSharedVarRightArmPosCreated)
            {
                if (!this.sharedVarRightArmPos.TryWrite(this.rightArm.CartesianPosition.v))
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't write " + this.sharedVarRightArmPos.Name);
            }
            else this.CreateRightArmPosSharedVar();
        }

        #endregion

        private void CheckSystemThreadTask()
        {
            bool isGeneralStatusUpdated = false;
            bool isLeftArmStatusUpdated = false;
            bool isRightArmStatusUpdated = false;
            bool areCmdAndCnnManStarted = false;

            while (this.status.Running && !isGeneralStatusUpdated)
            {
                if (!this.status.AreFilesLoaded)
                {
                    if (this.status.AreFilesLoaded = this.LoadMovsAndPositions())
                    {
                        this.taskPlanner = new TaskPlanner(this);
                        this.SetupCommandExecuters();
                        this.cnnMan.Start();
                        this.cmdMan.Start();
                        areCmdAndCnnManStarted = true;
                    }
                    else
                    {
                        Thread.Sleep(200);
                        continue;
                    }
                }

                if (this.status.AreVarsFromBBLoaded && areCmdAndCnnManStarted && !this.status.AreSharedVarsReady)
                {
                    if (!this.status.IsSharedVarLeftArmPosCreated) this.CreateLeftArmPosSharedVar();
                    if (!this.status.IsSharedVarRightArmPosCreated) this.CreateRightArmPosSharedVar();
                    if (!this.status.IsLowBatCreated) this.CreateLowBatSharedVar();
                }

                if (!isLeftArmStatusUpdated && (this.status.IsLeftArmReady = this.leftArm.IsManipulatorReady()))
                {
                    this.status.LeftArmStatus = "Ready";
                    this.OnStatusChanged(this.status);
                    isLeftArmStatusUpdated = true;
                }

                if (!isRightArmStatusUpdated && (this.status.IsRightArmReady = this.rightArm.IsManipulatorReady()))
                {
                    this.status.RightArmStatus = "Ready";
                    this.OnStatusChanged(this.status);
                    isRightArmStatusUpdated = true;
                }

                if (!isGeneralStatusUpdated && this.status.IsSystemReady)
                {
                    this.cmdMan.Ready = true;
                    this.OnStatusChanged(this.status);
                    isGeneralStatusUpdated = true;
                }

                

                Thread.Sleep(200);
            }
        }

        private void SetupCommandExecuters()
        {
			this.cmdArmsGoTo = new CmdArmsGoTo(this.taskPlanner);
			this.cmdArmsMove = new CmdArmsMove(this.taskPlanner);
			this.cmdArmsTorque = new CmdArmsTorque(this.taskPlanner);
            this.cmdArmsState = new CmdArmsState(this.taskPlanner);
            this.cmdGetVoltage = new CmdGetVoltage(this.taskPlanner);

            this.cmdRaState = new CmdRaState(this.taskPlanner);
            this.cmdRaAbsPos = new CmdRaAbsPos(this.taskPlanner);
            this.cmdRaArtPos = new CmdRaArtPos(this.taskPlanner);
            this.cmdRaGoTo = new CmdRaGoTo(this.taskPlanner);
            this.cmdRaMove = new CmdRaMove(this.taskPlanner);
            this.cmdRaOpenGripper = new CmdRaOpenGripper(this.taskPlanner);
            this.cmdRaCloseGripper = new CmdRaCloseGrip(this.taskPlanner);
            this.cmdRaReachable = new CmdRaReachable(this.taskPlanner);
            this.cmdRaRelPos = new CmdRaRelPos(this.taskPlanner);
            this.cmdRaTorque = new CmdRaTorque(this.taskPlanner);

            this.cmdLaState = new CmdLaState(this.taskPlanner);
            this.cmdLaAbsPos = new CmdLaAbsPos(this.taskPlanner);
            this.cmdLaArtPos = new CmdLaArtPos(this.taskPlanner);
            this.cmdLaGoTo = new CmdLaGoTo(this.taskPlanner);
            this.cmdLaMove = new CmdLaMove(this.taskPlanner);
            this.cmdLaOpenGripper = new CmdLaOpenGripper(this.taskPlanner);
            this.cmdLaCloseGripper = new CmdLaCloseGrip(this.taskPlanner);
            this.cmdLaReachable = new CmdLaReachable(this.taskPlanner);
            this.cmdLaRelPos = new CmdLaRelPos(this.taskPlanner);
            this.cmdLaTorque = new CmdLaTorque(this.taskPlanner);

			this.cmdMan.CommandExecuters.Add(this.cmdArmsGoTo);
			this.cmdMan.CommandExecuters.Add(this.cmdArmsMove);
			this.cmdMan.CommandExecuters.Add(this.cmdArmsTorque);
            this.cmdMan.CommandExecuters.Add(this.cmdArmsState);
            this.cmdMan.CommandExecuters.Add(this.cmdGetVoltage);

            this.cmdMan.CommandExecuters.Add(this.cmdRaState);
            this.cmdMan.CommandExecuters.Add(this.cmdRaAbsPos);
            this.cmdMan.CommandExecuters.Add(this.cmdRaArtPos);
            this.cmdMan.CommandExecuters.Add(this.cmdRaGoTo);			
			this.cmdMan.CommandExecuters.Add(this.cmdRaMove);
            this.cmdMan.CommandExecuters.Add(this.cmdRaOpenGripper);
            this.cmdMan.CommandExecuters.Add(this.cmdRaCloseGripper);
            this.cmdMan.CommandExecuters.Add(this.cmdRaReachable);
            this.cmdMan.CommandExecuters.Add(this.cmdRaRelPos);
            this.cmdMan.CommandExecuters.Add(this.cmdRaTorque);

            this.cmdMan.CommandExecuters.Add(this.cmdLaState);
            this.cmdMan.CommandExecuters.Add(this.cmdLaAbsPos);
            this.cmdMan.CommandExecuters.Add(this.cmdLaArtPos);
            this.cmdMan.CommandExecuters.Add(this.cmdLaGoTo);
            this.cmdMan.CommandExecuters.Add(this.cmdLaMove);
            this.cmdMan.CommandExecuters.Add(this.cmdLaOpenGripper);
            this.cmdMan.CommandExecuters.Add(this.cmdLaCloseGripper);
            this.cmdMan.CommandExecuters.Add(this.cmdLaReachable);
            this.cmdMan.CommandExecuters.Add(this.cmdLaRelPos);
            this.cmdMan.CommandExecuters.Add(this.cmdLaTorque);
        }

        public bool StopAllSystems()
        {
            this.cmdMan.Stop();
            this.cnnMan.Stop();

            return true;
        }

        public bool LoadMovsAndPositions()
        {
            PredefPosition[] tempPos = PredefPosition.DeserializeFromXml(this.fileLeftPredefPos);
            if (tempPos != null)
                foreach (PredefPosition pp in tempPos)
                    if (this.leftPredefPos.ContainsKey(pp.Name))
                        TextBoxStreamWriter.DefaultLog.WriteLine("Predefined Position repeated name: " + pp.Name);
                    else this.leftPredefPos.Add(pp.Name, pp);
            else TextBoxStreamWriter.DefaultLog.WriteLine("Cannot load left predefined positions from file: " + this.fileLeftPredefPos);

            tempPos = PredefPosition.DeserializeFromXml(this.fileRightPredefPos);
            if (tempPos != null)
                foreach (PredefPosition pp in tempPos)
                    if (this.rightPredefPos.ContainsKey(pp.Name))
                        TextBoxStreamWriter.DefaultLog.WriteLine("Predefined Position repeated name: " + pp.Name);
                    else this.rightPredefPos.Add(pp.Name, pp);
            else TextBoxStreamWriter.DefaultLog.WriteLine("Cannot load right predefined positions from file: " + this.fileLeftPredefPos);

            PredefMovement[] tempMovs = PredefMovement.DeserializeFromXml(this.fileLeftMovements);
            
			if (tempMovs != null)
				foreach (PredefMovement pm in tempMovs)
				{
					if (this.leftPredefMovs.ContainsKey(pm.Name))
						TextBoxStreamWriter.DefaultLog.WriteLine("Predefined Movement repeated name: " + pm.Name);
					else
					{
						List<int> toErase = new List<int>();

						for (int i = 0; i < pm.Positions.Count; i++)
						{

							if (this.leftPredefPos.ContainsKey(pm.Positions[i].Name))
								pm.Positions[i] = this.leftPredefPos[pm.Positions[i].Name];
							else
							{
								TextBoxStreamWriter.DefaultLog.WriteLine("leftArm: PredefPosition " + pm.Positions[i].Name + " on PredefMov " + pm.Name + " couldn´t be found on current PredefPos list");
								toErase.Add(i);
								//pm.Positions.RemoveAt(i);
							}
						}
						int count = 0;
						foreach (int num in toErase)
						{
							pm.Positions.RemoveAt(num-count);
							count++;
						}

						this.leftPredefMovs.Add(pm.Name, pm);
						
					}
				}
            else TextBoxStreamWriter.DefaultLog.WriteLine("Cannot load left predefined movements from file: " + this.fileLeftMovements);
			
			/*
			if(this.leftPredefMovs!=null)
				foreach(PredefMovement pm in (PredefMovement[])this.leftPredefMovs)
					foreach (PredefPosition pp in pm.Positions)
					{
						if (this.leftPredefPos.ContainsKey(pp.Name))
							pp = this.leftPredefPos[pp.Name];
						else
						{
							TextBoxStreamWriter.DefaultLog.WriteLine("leftArm: PredefPosition " + pp.Name + " on PredefMov " + pm.Name + " couldn´t be found on current PredefPos list");
							pm.Positions.Remove(pp.Name);
						}
					}
			*/

            tempMovs = PredefMovement.DeserializeFromXml(this.fileRightMovements);
            if (tempMovs != null)
				foreach (PredefMovement pm in tempMovs)
				{
					if (this.rightPredefMovs.ContainsKey(pm.Name))
						TextBoxStreamWriter.DefaultLog.WriteLine("Predefined Movement repeated name: " + pm.Name);
					else
					{
						List<int> toErase = new List<int>();

						for (int i = 0; i < pm.Positions.Count; i++)
						{

							if (this.rightPredefPos.ContainsKey(pm.Positions[i].Name))
								pm.Positions[i] = this.rightPredefPos[pm.Positions[i].Name];
							else
							{
								TextBoxStreamWriter.DefaultLog.WriteLine("rightArm: PredefPosition " + pm.Positions[i].Name + " on PredefMov " + pm.Name + " couldn´t be found on current PredefPos list");
								toErase.Add(i);
								//pm.Positions.RemoveAt(i);
							}
						}
						int count = 0;
						foreach (int num in toErase)
						{
							pm.Positions.RemoveAt(num - count);
							count++;
						}
						this.rightPredefMovs.Add(pm.Name, pm);

					}
				}
			else TextBoxStreamWriter.DefaultLog.WriteLine("Cannot load right predefined movements from file: " + this.fileRightMovements);
			/*
			if (this.righttPredefMovs != null)
				foreach (PredefMovement pm in this.rightPredefMovs)
					foreach (PredefPosition pp in pm.Positions)
					{
						if (this.rightPredefPos.ContainsKey(pp.Name))
							pp = this.rightPredefPos[pp.Name];
						else
						{
							TextBoxStreamWriter.DefaultLog.WriteLine("rightArm: PredefPosition " + pp.Name + " on PredefMov " + pm.Name + " couldn´t be found on current PredefPos list");
							pm.Positions.Remove(pp.Name);
						}
					}
			*/
			this.OnFilesLoaded(this.status);

            return true;
        }

        public bool SaveMovsAndPositions()
        {
            bool succes = true;
            succes &= PredefPosition.SerializeToXml(this.leftPredefPos.Values.ToArray(), this.fileLeftPredefPos);
            succes &= PredefPosition.SerializeToXml(this.rightPredefPos.Values.ToArray(), this.fileRightPredefPos);
            succes &= PredefMovement.SerializeToXml(this.leftPredefMovs.Values.ToArray(), this.fileLeftMovements);
            succes &= PredefMovement.SerializeToXml(this.rightPredefMovs.Values.ToArray(), this.fileRightMovements);
            
            return succes;
            //return true;
        }

		public bool AddPredefinedPosition(string name)
		{
			PredefPosition pp;
			if(this.rightPredefPos.ContainsKey(name))
				return false;
			else
			{
				pp = new PredefPosition(name, this.rightArm);
				this.rightPredefPos.Add(name, pp);
				return true;
			}
		}

		public bool AddPredefinedMovement(string name)
		{
			PredefMovement pm;
			if(this.rightPredefMovs.ContainsKey(name))
				return false;
			else
			{
				pm = new PredefMovement(name);
				this.rightPredefMovs.Add(name, pm);
				return true;
			}
		}

        #region Events

        private void OnStatusChanged(ArmControlStatus status)
        {
            if (this.StatusChanged != null) this.StatusChanged(status);
        }

        private void OnFilesLoaded(ArmControlStatus status)
        {
            if (this.FilesLoaded != null) this.FilesLoaded(status);
        }

        public event ArmControlStatusChangedEH StatusChanged;

        public event ArmControlStatusChangedEH FilesLoaded;

        private void leftArm_ArmPositionChanged(Manipulator senderArm)
        {
            this.UpdateLeftArmPosSharedVar();
        }

        private void rightArm_ArmPositionChanged(Manipulator senderArm)
        {
            this.UpdaterightArmPosSharedVar();
        }
        #endregion

        #region Properties

        public Manipulator LeftArm { get { return this.leftArm; } }

        public Manipulator RightArm { get { return this.rightArm; } }

        public SortedList<string, MapObstacle> MapObstacles { get { return this.mapObstacles; } }

        public SortedList<string, MapNode> MapNodes { get { return this.mapNodes; } }

        public SortedList<string, PredefPosition> LeftPredefPos { get { return this.leftPredefPos; } }

        public SortedList<string, PredefMovement> LeftPredefMovs { get { return this.leftPredefMovs; } }

        public SortedList<string, PredefPosition> RightPredefPos { get { return this.rightPredefPos; } }

        public SortedList<string, PredefMovement> RightPredefMovs { get { return this.rightPredefMovs; } }

        public MapOptimalPath OptLeftPath { get { return this.optLeftPath; } }

        public MapOptimalPath OptRightPath { get { return this.optRightPath; } }

        public MapGoalPoint LeftGoal { get { return this.leftGoal; } }

        public MapGoalPoint RightGoal { get { return this.rightGoal; } }

        #endregion
    }
}
