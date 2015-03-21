namespace Head
{
    partial class frmHead
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
			System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(frmHead));
			this.statusStrip1 = new System.Windows.Forms.StatusStrip();
			this.lblSysStat = new System.Windows.Forms.ToolStripStatusLabel();
			this.toolStripStatusLabel1 = new System.Windows.Forms.ToolStripStatusLabel();
			this.lblCnnBB = new System.Windows.Forms.ToolStripStatusLabel();
			this.toolStripStatusLabel2 = new System.Windows.Forms.ToolStripStatusLabel();
			this.lblMoving = new System.Windows.Forms.ToolStripStatusLabel();
			this.lblSerial = new System.Windows.Forms.Label();
			this.label1 = new System.Windows.Forms.Label();
			this.label2 = new System.Windows.Forms.Label();
			this.lblIDTilt = new System.Windows.Forms.Label();
			this.label3 = new System.Windows.Forms.Label();
			this.lblIDPan = new System.Windows.Forms.Label();
			this.label5 = new System.Windows.Forms.Label();
			this.lblGPan = new System.Windows.Forms.Label();
			this.label4 = new System.Windows.Forms.Label();
			this.lblGTilt = new System.Windows.Forms.Label();
			this.label6 = new System.Windows.Forms.Label();
			this.lblSpeed = new System.Windows.Forms.Label();
			this.groupBox1 = new System.Windows.Forms.GroupBox();
			this.txtLog = new System.Windows.Forms.TextBox();
			this.toolStrip1 = new System.Windows.Forms.ToolStrip();
			this.btnSettings = new System.Windows.Forms.ToolStripButton();
			this.txtParams = new System.Windows.Forms.ToolStripTextBox();
			this.statusStrip1.SuspendLayout();
			this.groupBox1.SuspendLayout();
			this.toolStrip1.SuspendLayout();
			this.SuspendLayout();
			// 
			// statusStrip1
			// 
			this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.lblSysStat,
            this.toolStripStatusLabel1,
            this.lblCnnBB,
            this.toolStripStatusLabel2,
            this.lblMoving});
			this.statusStrip1.Location = new System.Drawing.Point(0, 240);
			this.statusStrip1.Name = "statusStrip1";
			this.statusStrip1.Size = new System.Drawing.Size(337, 22);
			this.statusStrip1.TabIndex = 0;
			this.statusStrip1.Text = "statusStrip1";
			// 
			// lblSysStat
			// 
			this.lblSysStat.Font = new System.Drawing.Font("Segoe UI", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.lblSysStat.Name = "lblSysStat";
			this.lblSysStat.Size = new System.Drawing.Size(82, 17);
			this.lblSysStat.Text = "NOT READY";
			// 
			// toolStripStatusLabel1
			// 
			this.toolStripStatusLabel1.Name = "toolStripStatusLabel1";
			this.toolStripStatusLabel1.Size = new System.Drawing.Size(15, 17);
			this.toolStripStatusLabel1.Text = "||";
			// 
			// lblCnnBB
			// 
			this.lblCnnBB.Name = "lblCnnBB";
			this.lblCnnBB.Size = new System.Drawing.Size(73, 17);
			this.lblCnnBB.Text = "Not Conected";
			// 
			// toolStripStatusLabel2
			// 
			this.toolStripStatusLabel2.Name = "toolStripStatusLabel2";
			this.toolStripStatusLabel2.Size = new System.Drawing.Size(15, 17);
			this.toolStripStatusLabel2.Text = "||";
			// 
			// lblMoving
			// 
			this.lblMoving.Font = new System.Drawing.Font("Segoe UI", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.lblMoving.Name = "lblMoving";
			this.lblMoving.Size = new System.Drawing.Size(73, 17);
			this.lblMoving.Text = "Not Moving";
			// 
			// lblSerial
			// 
			this.lblSerial.AutoSize = true;
			this.lblSerial.Location = new System.Drawing.Point(7, 213);
			this.lblSerial.Name = "lblSerial";
			this.lblSerial.Size = new System.Drawing.Size(106, 13);
			this.lblSerial.TabIndex = 2;
			this.lblSerial.Text = "COM9, 250000 baud";
			// 
			// label1
			// 
			this.label1.AutoSize = true;
			this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.label1.Location = new System.Drawing.Point(7, 200);
			this.label1.Name = "label1";
			this.label1.Size = new System.Drawing.Size(66, 13);
			this.label1.TabIndex = 3;
			this.label1.Text = "Serial Port";
			// 
			// label2
			// 
			this.label2.AutoSize = true;
			this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.label2.Location = new System.Drawing.Point(162, 200);
			this.label2.Name = "label2";
			this.label2.Size = new System.Drawing.Size(38, 13);
			this.label2.TabIndex = 4;
			this.label2.Text = "IDTilt";
			// 
			// lblIDTilt
			// 
			this.lblIDTilt.AutoSize = true;
			this.lblIDTilt.Location = new System.Drawing.Point(162, 213);
			this.lblIDTilt.Name = "lblIDTilt";
			this.lblIDTilt.Size = new System.Drawing.Size(13, 13);
			this.lblIDTilt.TabIndex = 5;
			this.lblIDTilt.Text = "5";
			// 
			// label3
			// 
			this.label3.AutoSize = true;
			this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.label3.Location = new System.Drawing.Point(119, 200);
			this.label3.Name = "label3";
			this.label3.Size = new System.Drawing.Size(42, 13);
			this.label3.TabIndex = 6;
			this.label3.Text = "IDPan";
			// 
			// lblIDPan
			// 
			this.lblIDPan.AutoSize = true;
			this.lblIDPan.Location = new System.Drawing.Point(119, 213);
			this.lblIDPan.Name = "lblIDPan";
			this.lblIDPan.Size = new System.Drawing.Size(13, 13);
			this.lblIDPan.TabIndex = 7;
			this.lblIDPan.Text = "1";
			// 
			// label5
			// 
			this.label5.AutoSize = true;
			this.label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.label5.Location = new System.Drawing.Point(200, 200);
			this.label5.Name = "label5";
			this.label5.Size = new System.Drawing.Size(29, 13);
			this.label5.TabIndex = 8;
			this.label5.Text = "Pan";
			// 
			// lblGPan
			// 
			this.lblGPan.AutoSize = true;
			this.lblGPan.Location = new System.Drawing.Point(200, 213);
			this.lblGPan.Name = "lblGPan";
			this.lblGPan.Size = new System.Drawing.Size(13, 13);
			this.lblGPan.TabIndex = 9;
			this.lblGPan.Text = "0";
			// 
			// label4
			// 
			this.label4.AutoSize = true;
			this.label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.label4.Location = new System.Drawing.Point(232, 200);
			this.label4.Name = "label4";
			this.label4.Size = new System.Drawing.Size(25, 13);
			this.label4.TabIndex = 10;
			this.label4.Text = "Tilt";
			// 
			// lblGTilt
			// 
			this.lblGTilt.AutoSize = true;
			this.lblGTilt.Location = new System.Drawing.Point(232, 213);
			this.lblGTilt.Name = "lblGTilt";
			this.lblGTilt.Size = new System.Drawing.Size(13, 13);
			this.lblGTilt.TabIndex = 11;
			this.lblGTilt.Text = "0";
			// 
			// label6
			// 
			this.label6.AutoSize = true;
			this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.label6.Location = new System.Drawing.Point(259, 200);
			this.label6.Name = "label6";
			this.label6.Size = new System.Drawing.Size(66, 13);
			this.label6.TabIndex = 12;
			this.label6.Text = "MaxSpeed";
			// 
			// lblSpeed
			// 
			this.lblSpeed.AutoSize = true;
			this.lblSpeed.Location = new System.Drawing.Point(259, 213);
			this.lblSpeed.Name = "lblSpeed";
			this.lblSpeed.Size = new System.Drawing.Size(19, 13);
			this.lblSpeed.TabIndex = 13;
			this.lblSpeed.Text = "20";
			// 
			// groupBox1
			// 
			this.groupBox1.Controls.Add(this.txtLog);
			this.groupBox1.Location = new System.Drawing.Point(10, 28);
			this.groupBox1.Name = "groupBox1";
			this.groupBox1.Size = new System.Drawing.Size(307, 161);
			this.groupBox1.TabIndex = 14;
			this.groupBox1.TabStop = false;
			this.groupBox1.Text = "Output Log";
			// 
			// txtLog
			// 
			this.txtLog.BackColor = System.Drawing.Color.Black;
			this.txtLog.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
			this.txtLog.ForeColor = System.Drawing.Color.Lime;
			this.txtLog.Location = new System.Drawing.Point(6, 16);
			this.txtLog.Multiline = true;
			this.txtLog.Name = "txtLog";
			this.txtLog.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
			this.txtLog.Size = new System.Drawing.Size(295, 139);
			this.txtLog.TabIndex = 2;
			// 
			// toolStrip1
			// 
			this.toolStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.btnSettings,
            this.txtParams});
			this.toolStrip1.Location = new System.Drawing.Point(0, 0);
			this.toolStrip1.Name = "toolStrip1";
			this.toolStrip1.Size = new System.Drawing.Size(337, 25);
			this.toolStrip1.TabIndex = 15;
			this.toolStrip1.Text = "toolStrip1";
			// 
			// btnSettings
			// 
			this.btnSettings.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
			this.btnSettings.Image = ((System.Drawing.Image)(resources.GetObject("btnSettings.Image")));
			this.btnSettings.ImageTransparentColor = System.Drawing.Color.Magenta;
			this.btnSettings.Name = "btnSettings";
			this.btnSettings.Size = new System.Drawing.Size(23, 22);
			this.btnSettings.Text = "LookAt";
			this.btnSettings.Click += new System.EventHandler(this.btnSettings_Click);
			// 
			// txtParams
			// 
			this.txtParams.Name = "txtParams";
			this.txtParams.Size = new System.Drawing.Size(60, 25);
			this.txtParams.Text = "0 0 20";
			this.txtParams.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtParams_KeyPress);
			// 
			// frmHead
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size(337, 262);
			this.Controls.Add(this.toolStrip1);
			this.Controls.Add(this.groupBox1);
			this.Controls.Add(this.lblSpeed);
			this.Controls.Add(this.label6);
			this.Controls.Add(this.lblGTilt);
			this.Controls.Add(this.label4);
			this.Controls.Add(this.lblGPan);
			this.Controls.Add(this.label5);
			this.Controls.Add(this.lblIDPan);
			this.Controls.Add(this.label3);
			this.Controls.Add(this.lblIDTilt);
			this.Controls.Add(this.label2);
			this.Controls.Add(this.label1);
			this.Controls.Add(this.lblSerial);
			this.Controls.Add(this.statusStrip1);
			this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
			this.Name = "frmHead";
			this.Text = "HEAD";
			this.Load += new System.EventHandler(this.frmHead_Load);
			this.statusStrip1.ResumeLayout(false);
			this.statusStrip1.PerformLayout();
			this.groupBox1.ResumeLayout(false);
			this.groupBox1.PerformLayout();
			this.toolStrip1.ResumeLayout(false);
			this.toolStrip1.PerformLayout();
			this.ResumeLayout(false);
			this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.StatusStrip statusStrip1;
        private System.Windows.Forms.ToolStripStatusLabel lblSysStat;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel1;
        private System.Windows.Forms.ToolStripStatusLabel lblCnnBB;
        private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel2;
        private System.Windows.Forms.ToolStripStatusLabel lblMoving;
        private System.Windows.Forms.Label lblSerial;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label lblIDTilt;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label lblIDPan;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label lblGPan;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label lblGTilt;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label lblSpeed;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.TextBox txtLog;
        private System.Windows.Forms.ToolStrip toolStrip1;
        private System.Windows.Forms.ToolStripButton btnSettings;
        private System.Windows.Forms.ToolStripTextBox txtParams;
    }
}

