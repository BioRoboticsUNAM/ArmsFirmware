using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Head
{
    public class HeadStatus
    {
        public double GlobalPan;
        public double GlobalTilt;

        public double smPan;
        public double smTilt;
        public bool Move;

        public bool Running;

        public bool HeadIsReady;
        public bool IsMoving;
        public bool IsVariableCreated;
        public bool IsConnectedToBB;
        public bool VarsLoaded;
        

        public bool IsSystemReady
        {
            get 
            {
                return this.IsVariableCreated && this.VarsLoaded && this.IsConnectedToBB && this.HeadIsReady;
            }
        }

        public HeadStatus()
        {
            this.GlobalPan = 0;
            this.GlobalTilt = 0;
            this.smPan = 0;
            this.smTilt = 0;
            this.Running = false;
            this.HeadIsReady = false;
            this.IsMoving = false;
            this.IsVariableCreated = false;
            this.VarsLoaded = false;
            this.IsConnectedToBB = false;
            this.Move = false;
        }

        
    }
}
