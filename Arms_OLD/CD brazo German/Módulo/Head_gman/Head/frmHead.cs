using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using Robotics.Controls;

namespace Head
{
    public partial class frmHead : Form
    {
        private HeadManager headMan;
        private EHHeadStatus headStatusEventHandler;
        public frmHead()
        {
            InitializeComponent();
            if (!Directory.Exists(".\\" + DateTime.Today.ToString("yyyy-MM-dd")))
                Directory.CreateDirectory(".\\" + DateTime.Today.ToString("yyyy-MM-dd"));
            
            TextBoxStreamWriter.DefaultLog = new TextBoxStreamWriter(txtLog, ".\\" + DateTime.Today.ToString("yyyy-MM-dd") + "\\" +
                 DateTime.Now.ToString("HH.mm.ss") + ".txt", 500);

            TextBoxStreamWriter.DefaultLog.DefaultPriority = 0;
            TextBoxStreamWriter.DefaultLog.TextBoxVerbosityThreshold = 5;
        }

        private void btnSettings_Click(object sender, EventArgs e)
        {
            string[] parameters;
            char[] delimiters = {',',' '};
            if (txtParams.Text != "")
            {
                parameters = txtParams.Text.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);

                switch (parameters.Length)
                {
                    case 2:
                        this.headMan.taskMan.SetPosition(parameters[0], parameters[1]);
                        break;
                    case 3:
                        this.headMan.taskMan.SetPosSp(parameters[0], parameters[1], parameters[2]);
                        break;
                    default:
                        TextBoxStreamWriter.DefaultLog.WriteLine("frmHead: Incorrect number of parameters");
                        break;
                }
            }
        }

        private void frmHead_Load(object sender, EventArgs e)
        {
            TextBoxStreamWriter.DefaultLog.WriteLine("Execution Started " + DateTime.Today.ToString("yyyy-MM-dd") + " " + DateTime.Now.ToString("HH:mm:ss"));

			this.headStatusEventHandler = new EHHeadStatus(this.headStatusChanged);
            this.headMan = new HeadManager();
           
            this.headMan.HeadStatusChanged += this.headStatusEventHandler;

            lblIDPan.Text = this.headMan.settings.idpan.ToString();
            lblIDTilt.Text = this.headMan.settings.idtilt.ToString();
            lblSpeed.Text = this.headMan.settings.maxSpeedPerc.ToString("000");
            lblSerial.Text = this.headMan.settings.com + ", " + this.headMan.settings.baud + " baud";

            txtLog.Text = "";
            
        }

        private void headStatusChanged(HeadStatus status)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(this.headStatusEventHandler, status);
                return;
            }

            this.lblCnnBB.Text = status.IsConnectedToBB ? this.headMan.settings.port.ToString() : "Not Connected";
            this.lblGPan.Text = status.GlobalPan + "";
            this.lblGTilt.Text = status.GlobalTilt + "";

            this.lblMoving.ForeColor = status.IsMoving ? Color.GreenYellow : Color.Black;
            this.lblMoving.Text = status.IsMoving ? "MOVING" : "Not Moving";

            this.lblSysStat.ForeColor = status.IsSystemReady ? Color.Black : Color.Red;
            this.lblSysStat.Text = status.IsSystemReady ? "System READY" : "NOT READY";

            this.btnSettings.Enabled = status.HeadIsReady;
			this.Refresh();
        }

		private void txtParams_KeyPress(object sender, KeyPressEventArgs e)
		{
			if (e.KeyChar == 13) this.btnSettings.PerformClick();
		}
    }
}
