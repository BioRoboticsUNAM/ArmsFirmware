using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ManipulatorControl
{
    public class ArmControlStatus
    {
        public ArmControlStatus()
        {
            this.Running = false;
            this.AreFilesLoaded = false;
            this.IsConnectedToBB = false;
            this.IsLeftArmReady = false;
            this.LeftArmStatus = "Not Ready";
            this.IsRightArmReady = true;
            this.RightArmStatus = "Not Ready";
			this.LAVoltage = 0;
			this.RAVoltage = 0;
        }

		public double RAVoltage { get; set; }
		public double LAVoltage { get; set; }
        public bool Running { get; set; }

        public bool AreFilesLoaded { get; set; }

        public bool IsConnectedToBB { get; set; }
        public int CnnToBBPort { get; set; }
        public string CnnToBBStatus { get { return this.IsConnectedToBB ? this.CnnToBBPort.ToString() : "No Connected"; } }

        public bool AreVarsFromBBLoaded { get; set; }
        public bool IsSharedVarLeftArmPosCreated { get; set; }
        public bool IsSharedVarRightArmPosCreated { get; set; }
        public bool IsLowBatCreated { get; set; }
        public bool AreSharedVarsReady
        {
            get
            {
                return this.IsSharedVarLeftArmPosCreated && this.IsSharedVarRightArmPosCreated && this.IsLowBatCreated;
            }
        }

        public bool IsLeftArmReady { get; set; }
        public string LeftArmStatus { get; set; }
        public bool IsRightArmReady { get; set; }
        public string RightArmStatus { get; set; }

        public string LeftComPort { get; set; }
        public string RightComPort { get; set; }

        public bool IsSystemReady
        {
            get
            {
                return this.AreFilesLoaded  && this.IsConnectedToBB && this.IsLeftArmReady && this.IsRightArmReady;
            }
        }
        public string SystemStatus { get { return this.IsSystemReady ? "SYSTEM READY" : "SYSTEM NOT READY"; } }
    }
}
