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
using Robotics.Mathematics;

namespace ManipulatorControl
{
    public partial class frmArmsControl : Form
    {
        private ArmControlStatusChangedEH dlgStatusChanged;
        private ArmControlStatusChangedEH dlgFilesLoaded;
        private ManipulatorManager armsMan;
        

        public frmArmsControl()
        {
            InitializeComponent();

            if (!Directory.Exists(".\\" + DateTime.Today.ToString("yyyy-MM-dd")))
                Directory.CreateDirectory(".\\" + DateTime.Today.ToString("yyyy-MM-dd"));
            TextBoxStreamWriter.DefaultLog = new TextBoxStreamWriter(txtConsole, ".\\" + DateTime.Today.ToString("yyyy-MM-dd") + "\\" +
                 DateTime.Now.ToString("HH.mm.ss") + ".txt", 500);

            TextBoxStreamWriter.DefaultLog.DefaultPriority = 0;
            TextBoxStreamWriter.DefaultLog.TextBoxVerbosityThreshold = 9;
        }
        private void UpdatePosAndMovs()
        {
			//this.armsMan.LoadMovsAndPositions();

            this.lbLeftPredefPos.Items.Clear();
            this.lbLeftPredefMovs.Items.Clear();
            this.lbRightPredefPos.Items.Clear();
            this.lbRightPredefMovs.Items.Clear();
            
			foreach (PredefPosition pp in this.armsMan.LeftPredefPos.Values)
                this.lbLeftPredefPos.Items.Add(pp);
            foreach (PredefMovement pm in this.armsMan.LeftPredefMovs.Values)
                this.lbLeftPredefMovs.Items.Add(pm);
            foreach (PredefPosition pp in this.armsMan.RightPredefPos.Values)
                this.lbRightPredefPos.Items.Add(pp);
            foreach (PredefMovement pm in this.armsMan.RightPredefMovs.Values)
                this.lbRightPredefMovs.Items.Add(pm);

			

        }
        private void frmArmsControl_Load(object sender, EventArgs e)
        {
			TextBoxStreamWriter.DefaultLog.WriteLine("Compiled 11.06.12 1:44 a.m."); 

            this.lbLeftPredefPos.SelectedIndexChanged+=new EventHandler(this.listBox_Click);
            this.lbRightPredefPos.SelectedIndexChanged += new EventHandler(this.listBox_Click);
            this.lbRightPredefMovs.SelectedIndexChanged += new EventHandler(this.listBox_Click);
			this.lbLeftPredefMovs.SelectedIndexChanged+=new EventHandler(this.listBox_Click);

            this.armsMan = new ManipulatorManager(this.LeftArmPortName, this.RightArmPortName);
            this.dlgStatusChanged = new ArmControlStatusChangedEH(this.armsMan_StatusChanged);
            this.dlgFilesLoaded = new ArmControlStatusChangedEH(this.armsMan_FilesLoaded);
            this.armsMan.StatusChanged += this.dlgStatusChanged;
            this.armsMan.FilesLoaded += this.dlgFilesLoaded;

            
        }

        private void frmArmsControl_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (this.armsMan != null) this.armsMan.StopAllSystems();
        }

        private void armsMan_FilesLoaded(ArmControlStatus status)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(this.dlgFilesLoaded, status);
                return;
            }

            UpdatePosAndMovs();
            
        }

        private void armsMan_StatusChanged(ArmControlStatus status)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(this.dlgStatusChanged, status);
                return;
            }
            if (status.IsSystemReady) this.lblGeneralStatus.ForeColor = Color.Black;
            else this.lblGeneralStatus.ForeColor = Color.Red;

            this.lblGeneralStatus.Text = status.SystemStatus;
            this.lblCnnStatus.Text = "| BB: " + status.CnnToBBStatus + "   |";
            this.lblLeftArmStatus.Text = "LA: " + status.LeftArmStatus + "   |";
            this.lblRightArmStatus.Text = "RA: " + status.RightArmStatus + "   |";
            this.lblLeftComPort.Text = "LA: " + status.LeftComPort + "   |";
            this.lblRightComPort.Text = "RA: " + status.RightComPort + "   |";
        }

        private void listBox_Click(object sender, EventArgs e)
        {
            if (sender == null || !(sender is ListBox)) return;
            ListBox temp = (ListBox)(sender);
            this.pgMain.SelectedObject = temp.SelectedItem;

        }

        private void btnSaveAll_Click(object sender, EventArgs e)
        {
            
            if (this.armsMan.SaveMovsAndPositions())
                TextBoxStreamWriter.DefaultLog.WriteLine("BABAS.-> Files saved succesfully");
            else TextBoxStreamWriter.DefaultLog.WriteLine("BABAS.-> Can't save files");
            
            UpdatePosAndMovs();
            
        }

		

        #region Properties
        public string LeftArmPortName { get; set; }

        public string RightArmPortName { get; set; }
        #endregion

        #region Useless

        /*
        private void btnNLeft_Click(object sender, EventArgs e)
        {
            PredefPosition temp;

            if (rbLeft.Checked)
            {
                for (int n = 0; n < this.armsMan.LeftPredefPos.Count; n++)
                {
                    if (!this.armsMan.LeftPredefPos.ContainsKey("newPredefPos" + n))
                    {
                        temp = new PredefPosition("newPredefPos" + n, this.armsMan.LeftArm);
                        this.armsMan.LeftPredefPos.Add(temp.Name, temp);
                        break;
                    }
                }
                
            }
            else
            {

                for (int n = 0; n < this.armsMan.RightPredefPos.Count; n++)
                {
                    if (!this.armsMan.RightPredefPos.ContainsKey("newPredefPos" + n))
                    {
                        temp = new PredefPosition("newPredefPos" + n, this.armsMan.RightArm);
                        this.armsMan.RightPredefPos.Add(temp.Name, temp);
                        break;
                    }
                }
            }
            
            btnSaveAll.PerformClick();
            

        }

        private void btnDelPos_Click(object sender, EventArgs e)
        {
            PredefPosition toDel;
            
            if (rbLeft.Checked)
            {
                toDel = leftSel;
                leftSel = null;
            }
            else
            {
                toDel = rightSel;
                rightSel = null;
            }

            if (toDel == null) return;
            
            if (MessageBox.Show("Are you sure you want to delete the selected PredefPos: " + toDel.Name + " ?", "Delete PredefPosition", MessageBoxButtons.OKCancel,MessageBoxIcon.Exclamation) == DialogResult.OK)
            {
                if (rbLeft.Checked)
                    this.armsMan.LeftPredefPos.Remove(toDel.Name);
                else
                    this.armsMan.RightPredefPos.Remove(toDel.Name);
                
                TextBoxStreamWriter.DefaultLog.WriteLine("BABAS.-> Succesfully deleted PredefPosition "+ toDel.Name);
            }
            
            btnSaveAll.PerformClick();
            
            
            
        }

        private void btnSetPos_Click(object sender, EventArgs e)
        {
            Vector tempPos;

            if (rbLeft.Checked)
            {
                tempPos = armsMan.LeftArm.GetPositionArticular();

				if (tempPos != null)
				{
					leftSel.Q1 = tempPos[0];
					leftSel.Q2 = tempPos[1];
					leftSel.Q3 = tempPos[2];
					leftSel.Q4 = tempPos[3];
					leftSel.Q5 = tempPos[4];
					leftSel.Q6 = tempPos[5];
					leftSel.Q7 = tempPos[6];
					if (this.armsMan.LeftPredefPos.ContainsKey(leftSel.Name)) this.armsMan.LeftPredefPos.Remove(leftSel.Name);
					this.armsMan.LeftPredefPos.Add(leftSel.Name, leftSel);
				}
            }
            else
            {
                tempPos = armsMan.RightArm.GetPositionArticular();
				if (tempPos != null)
				{
					rightSel.Q1 = tempPos[0];
					rightSel.Q2 = tempPos[1];
					rightSel.Q3 = tempPos[2];
					rightSel.Q4 = tempPos[3];
					rightSel.Q5 = tempPos[4];
					rightSel.Q6 = tempPos[5];
					rightSel.Q7 = tempPos[6];
					if (this.armsMan.RightPredefPos.ContainsKey(rightSel.Name)) this.armsMan.RightPredefPos.Remove(rightSel.Name);
					this.armsMan.RightPredefPos.Add(rightSel.Name, rightSel);
				}
            }
            
            btnSaveAll.PerformClick();
        }*/
        #endregion

		private void btnSendT_Click(object sender, EventArgs e)
		{
			this.armsMan.LeftArm.TorqueOnOff(cbTorque.Checked);
			this.armsMan.RightArm.TorqueOnOff(cbTorque.Checked);
            cbTorque.Checked = !cbTorque.Checked;
        }
        
        #region PredefPosition Modification

        #region Left
        private void lbLeftPredefPos_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            PredefPosition temp;
            Vector tempPos;

            this.armsMan.LeftArm.TorqueOnOff(true);

            for (int n = 0; n < this.armsMan.LeftPredefPos.Count; n++)
            {
                if (!this.armsMan.LeftPredefPos.ContainsKey("newPredefPos" + n))
                {
                    temp = new PredefPosition("newPredefPos" + n, this.armsMan.LeftArm);

                    tempPos = armsMan.LeftArm.GetPositionArticular();

                    if (tempPos != null)
                    {
                        temp.Q1 = tempPos[0];
                        temp.Q2 = tempPos[1];
                        temp.Q3 = tempPos[2];
                        temp.Q4 = tempPos[3];
                        temp.Q5 = tempPos[4];
                        temp.Q6 = tempPos[5];
                        temp.Q7 = tempPos[6];
                    }

                    this.armsMan.LeftPredefPos.Add(temp.Name, temp);
                    break;
                }
            }
            
            btnSaveAll.PerformClick();
        }

        private void newPredefPositionToolStripMenuItem_Click(object sender, EventArgs e)
        {
            PredefPosition temp;

            for (int n = 0; n < this.armsMan.LeftPredefPos.Count; n++)
            {
                if (!this.armsMan.LeftPredefPos.ContainsKey("newPredefPos" + n))
                {
                    temp = new PredefPosition("newPredefPos" + n, this.armsMan.LeftArm);
                    this.armsMan.LeftPredefPos.Add(temp.Name, temp);
                    break;
                }
            }

            btnSaveAll.PerformClick();
        }

        private void deleteLeftPredefPosition_Click(object sender, EventArgs e)
        {
            PredefPosition toDel;
            toDel = (PredefPosition)lbLeftPredefPos.SelectedItem;

            if (toDel == null) return;

            if (MessageBox.Show("Are you sure you want to delete the selected PredefPos: " + toDel.Name + " ?", "Delete PredefPosition", MessageBoxButtons.OKCancel, MessageBoxIcon.Exclamation) == DialogResult.OK)
            {
                this.armsMan.LeftPredefPos.Remove(toDel.Name);
                TextBoxStreamWriter.DefaultLog.WriteLine("BABAS.-> Succesfully deleted PredefPosition " + toDel.Name);
            }

            btnSaveAll.PerformClick();


        }

        private void setLeftCurrentPredefPos_Click(object sender, EventArgs e)
        {
            Vector tempPos;
            tempPos = armsMan.LeftArm.GetPositionArticular();

            PredefPosition toSet;
            toSet = (PredefPosition)lbLeftPredefPos.SelectedItem;

            if (tempPos != null)
            {
                toSet.Q1 = tempPos[0];
                toSet.Q2 = tempPos[1];
                toSet.Q3 = tempPos[2];
                toSet.Q4 = tempPos[3];
                toSet.Q5 = tempPos[4];
                toSet.Q6 = tempPos[5];
                toSet.Q7 = tempPos[6];
                
                if (this.armsMan.LeftPredefPos.ContainsKey(toSet.Name)) this.armsMan.LeftPredefPos.Remove(toSet.Name);
                this.armsMan.LeftPredefPos.Add(toSet.Name, toSet);
            }

            btnSaveAll.PerformClick();

        }
        #endregion
        #region Right
        private void lbRightPredefPos_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            PredefPosition temp;
            Vector tempPos;

            this.armsMan.RightArm.TorqueOnOff(true);

            for (int n = 0; n < this.armsMan.RightPredefPos.Count; n++)
            {
                if (!this.armsMan.RightPredefPos.ContainsKey("newPredefPos" + n))
                {
                    temp = new PredefPosition("newPredefPos" + n, this.armsMan.RightArm);
                    tempPos = armsMan.RightArm.GetPositionArticular();

                    if (tempPos != null)
                    {
                        temp.Q1 = tempPos[0];
                        temp.Q2 = tempPos[1];
                        temp.Q3 = tempPos[2];
                        temp.Q4 = tempPos[3];
                        temp.Q5 = tempPos[4];
                        temp.Q6 = tempPos[5];
                        temp.Q7 = tempPos[6];
                    }

                    this.armsMan.RightPredefPos.Add(temp.Name, temp);
                    break;
                }
            }

            btnSaveAll.PerformClick();

        }

        private void deleteRightPredefPosition_Click(object sender, EventArgs e)
        {
            PredefPosition toDel;
            toDel = (PredefPosition)lbRightPredefPos.SelectedItem;

            if (toDel == null) return;

            if (MessageBox.Show("Are you sure you want to delete the selected PredefPos: " + toDel.Name + " ?", "Delete PredefPosition", MessageBoxButtons.OKCancel, MessageBoxIcon.Exclamation) == DialogResult.OK)
            {
                this.armsMan.RightPredefPos.Remove(toDel.Name);
                TextBoxStreamWriter.DefaultLog.WriteLine("BABAS.-> Succesfully deleted PredefPosition " + toDel.Name);
            }

            btnSaveAll.PerformClick();

        }

        private void newRightPredefPosition_Click(object sender, EventArgs e)
        {
            PredefPosition temp;

            for (int n = 0; n < this.armsMan.RightPredefPos.Count; n++)
            {
                if (!this.armsMan.RightPredefPos.ContainsKey("newPredefPos" + n))
                {
                    temp = new PredefPosition("newPredefPos" + n, this.armsMan.RightArm);
                    this.armsMan.RightPredefPos.Add(temp.Name, temp);
                    break;
                }
            }

            btnSaveAll.PerformClick();

        }

        private void setRightCurrentPredefPos_Click(object sender, EventArgs e)
        {
            Vector tempPos;
            tempPos = armsMan.RightArm.GetPositionArticular();

            PredefPosition toSet;
            toSet = (PredefPosition)lbRightPredefPos.SelectedItem;

            if (tempPos != null)
            {
                toSet.Q1 = tempPos[0];
                toSet.Q2 = tempPos[1];
                toSet.Q3 = tempPos[2];
                toSet.Q4 = tempPos[3];
                toSet.Q5 = tempPos[4];
                toSet.Q6 = tempPos[5];
                toSet.Q7 = tempPos[6];

                if (this.armsMan.RightPredefPos.ContainsKey(toSet.Name)) this.armsMan.RightPredefPos.Remove(toSet.Name);
                this.armsMan.RightPredefPos.Add(toSet.Name, toSet);
            }

            btnSaveAll.PerformClick();

        }
        #endregion

        private void tmBat_Tick(object sender, EventArgs e)
        {
            double LAv = 0, RAv = 0;
            if (!this.armsMan.taskPlanner.ArmsGetVoltage(out LAv, out RAv)) return;

            this.lblLAV.Text = LAv.ToString();
            this.lblLAV.ForeColor = LAv <= 16.0 ? Color.Red : Color.Black;
            this.lblRAV.Text = RAv.ToString();
            this.lblRAV.ForeColor = RAv <= 16.0 ? Color.Red : Color.Black;
            this.lblAlert.Visible = (RAv <= 16.0) || (LAv <= 16.0);


        }

        #endregion

        private void chb_useLaHand_CheckedChanged(object sender, EventArgs e)
        {
            this.chb_useLaHand.Enabled = false;
        }

        private void chb_useRaHand_CheckedChanged(object sender, EventArgs e)
        {
			if (this.chb_useRaHand.Checked)
				this.armsMan.taskPlanner.UseRaHand = true;
			else
				this.armsMan.taskPlanner.UseRaHand = false;
        }



 

    }
}
