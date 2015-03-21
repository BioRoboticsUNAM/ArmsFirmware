using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;
using System.IO.Ports;
using System.Xml.Serialization;
using Robotics.API;
using Robotics.Controls;
using Robotics.API.PrimitiveSharedVariables;


namespace Head
{
    public delegate void EHHeadStatus(HeadStatus status);
    public class HeadManager
    {
        private string moduleName = "HEAD";
        private int port;
        
        private ConnectionManager cnnMan;
        public CommandManager cmdMan;

        private Thread statusThread;

        public event EHHeadStatus HeadStatusChanged;
        public HeadStatus status;
        public Settings settings;
        public TaskMan taskMan;

        public DoubleArraySharedVariable hdPosition;

        private CmdLookAt cmdLookAt;
        private CmdLookAtRel cmdLookAtRel;
        private CmdLookAtStart cmdLookStart;
        private CmdLookAtStop cmdLookAtStop;

        public HeadManager()
        {
            this.status = new HeadStatus();
            if (!DeserializeFromXML("Settings.xml", out this.settings))
            {
                this.settings = new Settings();
                //SerializeToXML("Settings.xml", this.settings);
            }

			this.port = settings.port;
			this.cmdMan = new CommandManager();
            this.cnnMan = new ConnectionManager(this.moduleName, this.port, this.cmdMan);
            this.hdPosition = new DoubleArraySharedVariable("hd_pos");

            this.cmdMan.SharedVariablesLoaded += new SharedVariablesLoadedEventHandler(cmdMan_SharedVariablesLoaded);
            this.cnnMan.ClientConnected += new System.Net.Sockets.TcpClientConnectedEventHandler(cnnMan_ClientConnected);
            this.cnnMan.ClientDisconnected += new System.Net.Sockets.TcpClientDisconnectedEventHandler(cnnMan_ClientDisconnected);

            

            this.cnnMan.Start();
            this.cmdMan.Start();

            this.taskMan = new TaskMan(this);

            CreateSharedVariable();
			SetUpCmdExecuters();

            this.status.Running = true;
            this.statusThread = new Thread(new ThreadStart(StatusThreadTask));
            this.statusThread.IsBackground = true;
            this.statusThread.Start();

            TextBoxStreamWriter.DefaultLog.WriteLine("HeadManager: Succesfully started");

        }

        #region Status Thread
        private void StatusThreadTask()
        {
            while (this.status.Running && !this.status.IsSystemReady)
            {
                if (!this.status.IsSystemReady)
                {

                    if (!this.status.IsVariableCreated) this.status.IsVariableCreated = CreateSharedVariable();
                    if (!this.status.HeadIsReady) this.status.HeadIsReady = this.taskMan.head.InicializedHead();

                    
                }

                this.cmdMan.Ready = this.status.IsSystemReady;
				
				
                Thread.Sleep(200);

            }
			this.cmdMan.Ready = true;
        }

		public void OnHeadStatusChanged(HeadStatus st)
		{
			if (this.HeadStatusChanged != null)
				this.HeadStatusChanged(this.status);

		}
        #endregion

        #region CommandExecuters
        void SetUpCmdExecuters()
        {
            this.cmdLookAt = new CmdLookAt(this.taskMan);
            this.cmdLookAtRel = new CmdLookAtRel(this.taskMan);
            this.cmdLookStart = new CmdLookAtStart(this.taskMan);
            this.cmdLookAtStop = new CmdLookAtStop(this.taskMan);

            this.cmdMan.CommandExecuters.Add(this.cmdLookAt);
            this.cmdMan.CommandExecuters.Add(this.cmdLookAtRel);
            this.cmdMan.CommandExecuters.Add(this.cmdLookStart);
            this.cmdMan.CommandExecuters.Add(this.cmdLookAtStop);

        }
        #endregion

        #region Shared Vars
        public bool CreateSharedVariable()
        {
            try
            {
                if (!this.cmdMan.SharedVariables.Contains(hdPosition.Name))
                    this.cmdMan.SharedVariables.Add(this.hdPosition);
                else this.hdPosition = (DoubleArraySharedVariable)this.cmdMan.SharedVariables["hd_pos"];
                
                TextBoxStreamWriter.DefaultLog.WriteLine("Shared Variable hd_pos created succsfully");
                return true;
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Can't create shared variable");
                return false;
            }

        }

        public bool UpdatePositionSharedVariable()
        {
            if (this.hdPosition.Initialized)
            {

                if (this.hdPosition.TryWrite(new double[] { this.status.GlobalPan, this.status.GlobalTilt }))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head Manager: Shared variable succesfully written " + this.status.GlobalPan + " " + this.status.GlobalTilt);
                    return true;
                }
                else
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Head Manager: Shared variable cuoldn't be written");
                    return false;
                }

            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Head Manager: Shared variable not initialized");
                return false;
            }
        }

        public bool WriteSharedVariable()
        {
            if (!this.status.IsVariableCreated)
            {
                this.status.IsVariableCreated = CreateSharedVariable();
            }

            if (!this.status.IsVariableCreated)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Shared Variable couldn't be created");
                return false;
            }

            if (this.hdPosition.TryWrite(new double[] { this.status.GlobalPan, this.status.GlobalTilt }))
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("hd_pos Written succesfully ");
                return true;
            }
            else
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Couldn't write shared variable hd_pos");
                return false;
            }
            
        }
        #endregion

        #region EmptyEventHandler
        public void hdStatusChanged(HeadStatus stat)
		{
			return;
		}
        #endregion

        #region BlackBoard Methods
        private void cmdMan_SharedVariablesLoaded(CommandManager commandMan)
        {
            
			if (cmdMan.SharedVariables.Contains(hdPosition.Name))
				hdPosition = (DoubleArraySharedVariable)cmdMan.SharedVariables[hdPosition.Name];
			else
				cmdMan.SharedVariables.Add(hdPosition);

            this.status.VarsLoaded = true;
            this.HeadStatusChanged(this.status);
        }

        private void cnnMan_ClientConnected(System.Net.Sockets.Socket s)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("HeadManager: Client connected PORT " + this.port);
            this.status.IsConnectedToBB = true;
            if(this.HeadStatusChanged!=null)
				this.HeadStatusChanged(this.status);
        }

        private void cnnMan_ClientDisconnected(System.Net.EndPoint ep)
        {
            //When the client disconnects every status variable is turnedOff
            TextBoxStreamWriter.DefaultLog.WriteLine("HeadManager: Client disconnected");
            this.status.IsConnectedToBB = false;
            if(this.HeadStatusChanged!=null)
				this.HeadStatusChanged(this.status);
        }
        #endregion

        #region Settings Methods
        private bool SerializeToXML(string path, Settings set)
        {
            //Genera un error raro si no existe el archivo, no usar  hasta corregirlo...
            XmlSerializer serializer = new XmlSerializer(typeof(Settings));
            if (File.Exists(path))
            {
                try
                {
                    string[] emptyStrings = { "" };
                    File.WriteAllLines(path, emptyStrings);
                }
                catch(Exception e)
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Can't erase file " + e.ToString());
                }
                
            }
            else
            {
                File.Create(path);

                if (!File.Exists(path))
                {
                    TextBoxStreamWriter.DefaultLog.WriteLine("Couldn't create Settings file");
                    return false;
                }
                else TextBoxStreamWriter.DefaultLog.WriteLine("Settings file succesfully created");
            }


            try
            {

                Stream stream = File.OpenWrite(path);
                serializer.Serialize(stream, set);
                stream.Close();
                TextBoxStreamWriter.DefaultLog.WriteLine("Settings correctly saved to file: Settings.xml");
            }
            catch(Exception e)
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("Settings couldn't be saved " + e.ToString());
                return false;
            }
            return true;
        }
        private bool DeserializeFromXML(string Path, out Settings extractedSettings)
        {
            //Funciona bien
            extractedSettings = new Settings();
            try
            {
                XmlSerializer serializer = new XmlSerializer(typeof(Settings));

                Stream stream = File.OpenRead(Path);
                extractedSettings = (Settings)serializer.Deserialize(stream);
                stream.Close();
                TextBoxStreamWriter.DefaultLog.WriteLine("HeadManager: Succesfully loaded settings from file");
                return true;
            }
            catch
            {
                TextBoxStreamWriter.DefaultLog.WriteLine("HeadManager: Can't load settings from file, using default settings");
                return false;
            }
        }
        #endregion
    }
}
