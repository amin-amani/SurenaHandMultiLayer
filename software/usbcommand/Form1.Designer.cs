namespace usbcommand
{
    partial class frmmain
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
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(frmmain));
            this.btnfind = new System.Windows.Forms.Button();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.lab_status = new System.Windows.Forms.Label();
            this.btn_can1send = new System.Windows.Forms.Button();
            this.btn_can1read = new System.Windows.Forms.Button();
            this.set_baud1 = new System.Windows.Forms.Button();
            this.dataA0 = new System.Windows.Forms.TextBox();
            this.dataA1 = new System.Windows.Forms.TextBox();
            this.dataA2 = new System.Windows.Forms.TextBox();
            this.dataA3 = new System.Windows.Forms.TextBox();
            this.dataB0 = new System.Windows.Forms.TextBox();
            this.dataB1 = new System.Windows.Forms.TextBox();
            this.dataB2 = new System.Windows.Forms.TextBox();
            this.dataB3 = new System.Windows.Forms.TextBox();
            this.can2_send = new System.Windows.Forms.Button();
            this.dataA10 = new System.Windows.Forms.TextBox();
            this.dataA11 = new System.Windows.Forms.TextBox();
            this.dataA12 = new System.Windows.Forms.TextBox();
            this.dataA13 = new System.Windows.Forms.TextBox();
            this.dataB10 = new System.Windows.Forms.TextBox();
            this.dataB11 = new System.Windows.Forms.TextBox();
            this.dataB12 = new System.Windows.Forms.TextBox();
            this.dataB13 = new System.Windows.Forms.TextBox();
            this.cmb_baud = new System.Windows.Forms.ComboBox();
            this.set_baud2 = new System.Windows.Forms.Button();
            this.cmb_baud2 = new System.Windows.Forms.ComboBox();
            this.txt_can1_id = new System.Windows.Forms.TextBox();
            this.txt_can2_id = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label12 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.btn_text_clear = new System.Windows.Forms.Button();
            this.btn_auto_read = new System.Windows.Forms.Button();
            this.skinEngine1 = new Sunisoft.IrisSkin.SkinEngine(((System.ComponentModel.Component)(this)));
            this.txt_recieved_data = new System.Windows.Forms.RichTextBox();
            this.btn_get_adc = new System.Windows.Forms.Button();
            this.label16 = new System.Windows.Forms.Label();
            this.lab_adc_disp0 = new System.Windows.Forms.Label();
            this.trk_motor_position = new System.Windows.Forms.TrackBar();
            this.num_motor_index = new System.Windows.Forms.NumericUpDown();
            this.btn_stop_all_motors = new System.Windows.Forms.Button();
            this.num_motor_position = new System.Windows.Forms.NumericUpDown();
            this.label19 = new System.Windows.Forms.Label();
            this.label20 = new System.Windows.Forms.Label();
            this.Btn_set_position = new System.Windows.Forms.Button();
            this.num_set_position = new System.Windows.Forms.NumericUpDown();
            this.groupBox1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trk_motor_position)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_motor_index)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_motor_position)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_set_position)).BeginInit();
            this.SuspendLayout();
            // 
            // btnfind
            // 
            this.btnfind.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F);
            this.btnfind.Location = new System.Drawing.Point(11, 46);
            this.btnfind.Margin = new System.Windows.Forms.Padding(2);
            this.btnfind.Name = "btnfind";
            this.btnfind.Size = new System.Drawing.Size(204, 46);
            this.btnfind.TabIndex = 0;
            this.btnfind.Text = "Device Search";
            this.btnfind.UseVisualStyleBackColor = true;
            this.btnfind.Click += new System.EventHandler(this.btnfind_Click);
            // 
            // timer1
            // 
            this.timer1.Interval = 10;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // lab_status
            // 
            this.lab_status.AutoSize = true;
            this.lab_status.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F);
            this.lab_status.Location = new System.Drawing.Point(12, 400);
            this.lab_status.Name = "lab_status";
            this.lab_status.Size = new System.Drawing.Size(24, 25);
            this.lab_status.TabIndex = 13;
            this.lab_status.Text = "_";
            // 
            // btn_can1send
            // 
            this.btn_can1send.Location = new System.Drawing.Point(61, 638);
            this.btn_can1send.Name = "btn_can1send";
            this.btn_can1send.Size = new System.Drawing.Size(75, 23);
            this.btn_can1send.TabIndex = 19;
            this.btn_can1send.Text = "CAN1Send";
            this.btn_can1send.UseVisualStyleBackColor = true;
            this.btn_can1send.Click += new System.EventHandler(this.btn_can1send_Click);
            // 
            // btn_can1read
            // 
            this.btn_can1read.Location = new System.Drawing.Point(61, 606);
            this.btn_can1read.Name = "btn_can1read";
            this.btn_can1read.Size = new System.Drawing.Size(75, 23);
            this.btn_can1read.TabIndex = 20;
            this.btn_can1read.Text = "CAN Read";
            this.btn_can1read.UseVisualStyleBackColor = true;
            this.btn_can1read.Click += new System.EventHandler(this.btn_can1read_Click);
            // 
            // set_baud1
            // 
            this.set_baud1.Location = new System.Drawing.Point(150, 584);
            this.set_baud1.Name = "set_baud1";
            this.set_baud1.Size = new System.Drawing.Size(75, 23);
            this.set_baud1.TabIndex = 22;
            this.set_baud1.Text = "SetBaud1";
            this.set_baud1.UseVisualStyleBackColor = true;
            this.set_baud1.Click += new System.EventHandler(this.button2_Click);
            // 
            // dataA0
            // 
            this.dataA0.Location = new System.Drawing.Point(111, 52);
            this.dataA0.Name = "dataA0";
            this.dataA0.Size = new System.Drawing.Size(40, 20);
            this.dataA0.TabIndex = 24;
            this.dataA0.Text = "1";
            // 
            // dataA1
            // 
            this.dataA1.Location = new System.Drawing.Point(157, 52);
            this.dataA1.Name = "dataA1";
            this.dataA1.Size = new System.Drawing.Size(43, 20);
            this.dataA1.TabIndex = 25;
            this.dataA1.Text = "2";
            // 
            // dataA2
            // 
            this.dataA2.Location = new System.Drawing.Point(206, 52);
            this.dataA2.Name = "dataA2";
            this.dataA2.Size = new System.Drawing.Size(42, 20);
            this.dataA2.TabIndex = 26;
            this.dataA2.Text = "3";
            // 
            // dataA3
            // 
            this.dataA3.Location = new System.Drawing.Point(255, 52);
            this.dataA3.Name = "dataA3";
            this.dataA3.Size = new System.Drawing.Size(35, 20);
            this.dataA3.TabIndex = 27;
            this.dataA3.Text = "4";
            // 
            // dataB0
            // 
            this.dataB0.Location = new System.Drawing.Point(296, 52);
            this.dataB0.Name = "dataB0";
            this.dataB0.Size = new System.Drawing.Size(40, 20);
            this.dataB0.TabIndex = 28;
            this.dataB0.Text = "5";
            // 
            // dataB1
            // 
            this.dataB1.Location = new System.Drawing.Point(343, 52);
            this.dataB1.Name = "dataB1";
            this.dataB1.Size = new System.Drawing.Size(42, 20);
            this.dataB1.TabIndex = 29;
            this.dataB1.Text = "6";
            // 
            // dataB2
            // 
            this.dataB2.Location = new System.Drawing.Point(391, 52);
            this.dataB2.Name = "dataB2";
            this.dataB2.Size = new System.Drawing.Size(42, 20);
            this.dataB2.TabIndex = 30;
            this.dataB2.Text = "7";
            // 
            // dataB3
            // 
            this.dataB3.Location = new System.Drawing.Point(440, 52);
            this.dataB3.Name = "dataB3";
            this.dataB3.Size = new System.Drawing.Size(35, 20);
            this.dataB3.TabIndex = 31;
            this.dataB3.Text = "8";
            // 
            // can2_send
            // 
            this.can2_send.Location = new System.Drawing.Point(61, 674);
            this.can2_send.Name = "can2_send";
            this.can2_send.Size = new System.Drawing.Size(75, 23);
            this.can2_send.TabIndex = 32;
            this.can2_send.Text = "CAN2Send";
            this.can2_send.UseVisualStyleBackColor = true;
            this.can2_send.Click += new System.EventHandler(this.can2_send_Click);
            // 
            // dataA10
            // 
            this.dataA10.Location = new System.Drawing.Point(111, 95);
            this.dataA10.Name = "dataA10";
            this.dataA10.Size = new System.Drawing.Size(40, 20);
            this.dataA10.TabIndex = 33;
            this.dataA10.Text = "11";
            // 
            // dataA11
            // 
            this.dataA11.Location = new System.Drawing.Point(157, 95);
            this.dataA11.Name = "dataA11";
            this.dataA11.Size = new System.Drawing.Size(43, 20);
            this.dataA11.TabIndex = 34;
            this.dataA11.Text = "12";
            // 
            // dataA12
            // 
            this.dataA12.Location = new System.Drawing.Point(206, 95);
            this.dataA12.Name = "dataA12";
            this.dataA12.Size = new System.Drawing.Size(42, 20);
            this.dataA12.TabIndex = 35;
            this.dataA12.Text = "13";
            // 
            // dataA13
            // 
            this.dataA13.Location = new System.Drawing.Point(255, 95);
            this.dataA13.Name = "dataA13";
            this.dataA13.Size = new System.Drawing.Size(35, 20);
            this.dataA13.TabIndex = 36;
            this.dataA13.Text = "14";
            // 
            // dataB10
            // 
            this.dataB10.Location = new System.Drawing.Point(296, 97);
            this.dataB10.Name = "dataB10";
            this.dataB10.Size = new System.Drawing.Size(40, 20);
            this.dataB10.TabIndex = 37;
            this.dataB10.Text = "15";
            // 
            // dataB11
            // 
            this.dataB11.Location = new System.Drawing.Point(342, 97);
            this.dataB11.Name = "dataB11";
            this.dataB11.Size = new System.Drawing.Size(43, 20);
            this.dataB11.TabIndex = 38;
            this.dataB11.Text = "16";
            // 
            // dataB12
            // 
            this.dataB12.Location = new System.Drawing.Point(391, 97);
            this.dataB12.Name = "dataB12";
            this.dataB12.Size = new System.Drawing.Size(42, 20);
            this.dataB12.TabIndex = 39;
            this.dataB12.Text = "17";
            // 
            // dataB13
            // 
            this.dataB13.Location = new System.Drawing.Point(440, 97);
            this.dataB13.Name = "dataB13";
            this.dataB13.Size = new System.Drawing.Size(35, 20);
            this.dataB13.TabIndex = 40;
            this.dataB13.Text = "18";
            // 
            // cmb_baud
            // 
            this.cmb_baud.FormattingEnabled = true;
            this.cmb_baud.Items.AddRange(new object[] {
            "1000000",
            "128000"});
            this.cmb_baud.Location = new System.Drawing.Point(240, 584);
            this.cmb_baud.Name = "cmb_baud";
            this.cmb_baud.Size = new System.Drawing.Size(121, 21);
            this.cmb_baud.TabIndex = 41;
            this.cmb_baud.Text = "1000000";
            // 
            // set_baud2
            // 
            this.set_baud2.Location = new System.Drawing.Point(150, 609);
            this.set_baud2.Name = "set_baud2";
            this.set_baud2.Size = new System.Drawing.Size(75, 23);
            this.set_baud2.TabIndex = 45;
            this.set_baud2.Text = "SetBaud2";
            this.set_baud2.UseVisualStyleBackColor = true;
            this.set_baud2.Click += new System.EventHandler(this.set_baud2_Click);
            // 
            // cmb_baud2
            // 
            this.cmb_baud2.FormattingEnabled = true;
            this.cmb_baud2.Items.AddRange(new object[] {
            "1000000"});
            this.cmb_baud2.Location = new System.Drawing.Point(240, 612);
            this.cmb_baud2.Name = "cmb_baud2";
            this.cmb_baud2.Size = new System.Drawing.Size(121, 21);
            this.cmb_baud2.TabIndex = 46;
            this.cmb_baud2.Text = "1000000";
            // 
            // txt_can1_id
            // 
            this.txt_can1_id.Location = new System.Drawing.Point(54, 51);
            this.txt_can1_id.Name = "txt_can1_id";
            this.txt_can1_id.Size = new System.Drawing.Size(51, 20);
            this.txt_can1_id.TabIndex = 47;
            this.txt_can1_id.Text = "281";
            // 
            // txt_can2_id
            // 
            this.txt_can2_id.Location = new System.Drawing.Point(54, 94);
            this.txt_can2_id.Name = "txt_can2_id";
            this.txt_can2_id.Size = new System.Drawing.Size(51, 20);
            this.txt_can2_id.TabIndex = 48;
            this.txt_can2_id.Text = "381";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label12);
            this.groupBox1.Controls.Add(this.label9);
            this.groupBox1.Controls.Add(this.label8);
            this.groupBox1.Controls.Add(this.label7);
            this.groupBox1.Controls.Add(this.label6);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label11);
            this.groupBox1.Controls.Add(this.label10);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.txt_can2_id);
            this.groupBox1.Controls.Add(this.dataA0);
            this.groupBox1.Controls.Add(this.txt_can1_id);
            this.groupBox1.Controls.Add(this.dataA1);
            this.groupBox1.Controls.Add(this.dataA2);
            this.groupBox1.Controls.Add(this.dataA3);
            this.groupBox1.Controls.Add(this.dataB0);
            this.groupBox1.Controls.Add(this.dataB1);
            this.groupBox1.Controls.Add(this.dataB13);
            this.groupBox1.Controls.Add(this.dataB2);
            this.groupBox1.Controls.Add(this.dataB12);
            this.groupBox1.Controls.Add(this.dataB3);
            this.groupBox1.Controls.Add(this.dataB11);
            this.groupBox1.Controls.Add(this.dataA10);
            this.groupBox1.Controls.Add(this.dataB10);
            this.groupBox1.Controls.Add(this.dataA11);
            this.groupBox1.Controls.Add(this.dataA13);
            this.groupBox1.Controls.Add(this.dataA12);
            this.groupBox1.Location = new System.Drawing.Point(174, 752);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(487, 168);
            this.groupBox1.TabIndex = 49;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "SEND";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(54, 149);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(142, 13);
            this.label12.TabIndex = 51;
            this.label12.Text = "Attention:All Values Are Hex!";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(440, 26);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(33, 13);
            this.label9.TabIndex = 50;
            this.label9.Text = "byte7";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(393, 26);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(33, 13);
            this.label8.TabIndex = 50;
            this.label8.Text = "byte6";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(346, 26);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(33, 13);
            this.label7.TabIndex = 50;
            this.label7.Text = "byte5";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(299, 26);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(33, 13);
            this.label6.TabIndex = 50;
            this.label6.Text = "byte4";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(252, 26);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(33, 13);
            this.label5.TabIndex = 50;
            this.label5.Text = "byte3";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(205, 26);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(33, 13);
            this.label4.TabIndex = 50;
            this.label4.Text = "byte2";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(158, 26);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(33, 13);
            this.label3.TabIndex = 50;
            this.label3.Text = "byte1";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(111, 26);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(33, 13);
            this.label2.TabIndex = 50;
            this.label2.Text = "byte0";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(6, 58);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(35, 13);
            this.label11.TabIndex = 49;
            this.label11.Text = "CAN1";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(6, 97);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(35, 13);
            this.label10.TabIndex = 49;
            this.label10.Text = "CAN2";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(69, 26);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(18, 13);
            this.label1.TabIndex = 49;
            this.label1.Text = "ID";
            // 
            // btn_text_clear
            // 
            this.btn_text_clear.Location = new System.Drawing.Point(601, 560);
            this.btn_text_clear.Name = "btn_text_clear";
            this.btn_text_clear.Size = new System.Drawing.Size(60, 23);
            this.btn_text_clear.TabIndex = 50;
            this.btn_text_clear.Text = "Clear";
            this.btn_text_clear.UseVisualStyleBackColor = true;
            this.btn_text_clear.Click += new System.EventHandler(this.btn_text_clear_Click);
            // 
            // btn_auto_read
            // 
            this.btn_auto_read.Location = new System.Drawing.Point(847, 482);
            this.btn_auto_read.Name = "btn_auto_read";
            this.btn_auto_read.Size = new System.Drawing.Size(75, 44);
            this.btn_auto_read.TabIndex = 52;
            this.btn_auto_read.Text = "Auto Read OFF";
            this.btn_auto_read.UseVisualStyleBackColor = true;
            this.btn_auto_read.Click += new System.EventHandler(this.btn_auto_read_Click_1);
            // 
            // skinEngine1
            // 
            this.skinEngine1.SerialNumber = "";
            this.skinEngine1.SkinFile = null;
            // 
            // txt_recieved_data
            // 
            this.txt_recieved_data.Location = new System.Drawing.Point(301, 599);
            this.txt_recieved_data.Name = "txt_recieved_data";
            this.txt_recieved_data.Size = new System.Drawing.Size(371, 133);
            this.txt_recieved_data.TabIndex = 54;
            this.txt_recieved_data.Text = "";
            // 
            // btn_get_adc
            // 
            this.btn_get_adc.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F);
            this.btn_get_adc.Location = new System.Drawing.Point(17, 113);
            this.btn_get_adc.Name = "btn_get_adc";
            this.btn_get_adc.Size = new System.Drawing.Size(148, 54);
            this.btn_get_adc.TabIndex = 55;
            this.btn_get_adc.Text = "Read ADC";
            this.btn_get_adc.UseVisualStyleBackColor = true;
            this.btn_get_adc.Click += new System.EventHandler(this.btn_get_adc_Click);
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label16.Location = new System.Drawing.Point(201, 113);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(128, 25);
            this.label16.TabIndex = 59;
            this.label16.Text = "ADC Values";
            // 
            // lab_adc_disp0
            // 
            this.lab_adc_disp0.AutoSize = true;
            this.lab_adc_disp0.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lab_adc_disp0.Location = new System.Drawing.Point(186, 138);
            this.lab_adc_disp0.Name = "lab_adc_disp0";
            this.lab_adc_disp0.Size = new System.Drawing.Size(114, 25);
            this.lab_adc_disp0.TabIndex = 65;
            this.lab_adc_disp0.Text = "0 0 0 0 0 0";
            // 
            // trk_motor_position
            // 
            this.trk_motor_position.Location = new System.Drawing.Point(17, 275);
            this.trk_motor_position.Maximum = 1900;
            this.trk_motor_position.Minimum = -1900;
            this.trk_motor_position.Name = "trk_motor_position";
            this.trk_motor_position.Size = new System.Drawing.Size(283, 45);
            this.trk_motor_position.TabIndex = 68;
            this.trk_motor_position.Scroll += new System.EventHandler(this.trk_motor_position_Scroll);
            // 
            // num_motor_index
            // 
            this.num_motor_index.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.num_motor_index.Location = new System.Drawing.Point(331, 185);
            this.num_motor_index.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.num_motor_index.Name = "num_motor_index";
            this.num_motor_index.Size = new System.Drawing.Size(76, 31);
            this.num_motor_index.TabIndex = 69;
            // 
            // btn_stop_all_motors
            // 
            this.btn_stop_all_motors.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_stop_all_motors.Location = new System.Drawing.Point(335, 269);
            this.btn_stop_all_motors.Name = "btn_stop_all_motors";
            this.btn_stop_all_motors.Size = new System.Drawing.Size(97, 51);
            this.btn_stop_all_motors.TabIndex = 70;
            this.btn_stop_all_motors.Text = "Stop All";
            this.btn_stop_all_motors.UseVisualStyleBackColor = true;
            this.btn_stop_all_motors.Click += new System.EventHandler(this.btn_stop_all_motors_Click);
            // 
            // num_motor_position
            // 
            this.num_motor_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F);
            this.num_motor_position.Location = new System.Drawing.Point(331, 222);
            this.num_motor_position.Maximum = new decimal(new int[] {
            1900,
            0,
            0,
            0});
            this.num_motor_position.Minimum = new decimal(new int[] {
            1900,
            0,
            0,
            -2147483648});
            this.num_motor_position.Name = "num_motor_position";
            this.num_motor_position.Size = new System.Drawing.Size(76, 31);
            this.num_motor_position.TabIndex = 71;
            this.num_motor_position.Value = new decimal(new int[] {
            1200,
            0,
            0,
            0});
            this.num_motor_position.ValueChanged += new System.EventHandler(this.num_motor_position_ValueChanged);
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label19.Location = new System.Drawing.Point(40, 191);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(125, 25);
            this.label19.TabIndex = 72;
            this.label19.Text = "Motor Index";
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label20.Location = new System.Drawing.Point(40, 220);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(71, 25);
            this.label20.TabIndex = 73;
            this.label20.Text = "speed";
            // 
            // Btn_set_position
            // 
            this.Btn_set_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.Btn_set_position.Location = new System.Drawing.Point(30, 337);
            this.Btn_set_position.Name = "Btn_set_position";
            this.Btn_set_position.Size = new System.Drawing.Size(135, 51);
            this.Btn_set_position.TabIndex = 74;
            this.Btn_set_position.Text = "Set position";
            this.Btn_set_position.UseVisualStyleBackColor = true;
            this.Btn_set_position.Click += new System.EventHandler(this.Btn_set_position_Click);
            // 
            // num_set_position
            // 
            this.num_set_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F);
            this.num_set_position.Location = new System.Drawing.Point(331, 348);
            this.num_set_position.Maximum = new decimal(new int[] {
            4096,
            0,
            0,
            0});
            this.num_set_position.Name = "num_set_position";
            this.num_set_position.Size = new System.Drawing.Size(76, 31);
            this.num_set_position.TabIndex = 75;
            this.num_set_position.Value = new decimal(new int[] {
            1200,
            0,
            0,
            0});
            // 
            // frmmain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(568, 463);
            this.Controls.Add(this.num_set_position);
            this.Controls.Add(this.Btn_set_position);
            this.Controls.Add(this.label20);
            this.Controls.Add(this.label19);
            this.Controls.Add(this.num_motor_position);
            this.Controls.Add(this.btn_stop_all_motors);
            this.Controls.Add(this.num_motor_index);
            this.Controls.Add(this.trk_motor_position);
            this.Controls.Add(this.lab_adc_disp0);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.btn_get_adc);
            this.Controls.Add(this.txt_recieved_data);
            this.Controls.Add(this.btn_auto_read);
            this.Controls.Add(this.btn_text_clear);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.cmb_baud2);
            this.Controls.Add(this.set_baud2);
            this.Controls.Add(this.cmb_baud);
            this.Controls.Add(this.can2_send);
            this.Controls.Add(this.set_baud1);
            this.Controls.Add(this.btn_can1read);
            this.Controls.Add(this.btn_can1send);
            this.Controls.Add(this.lab_status);
            this.Controls.Add(this.btnfind);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(2);
            this.MaximizeBox = false;
            this.Name = "frmmain";
            this.Text = "Surena hand control v1.0";
            this.Load += new System.EventHandler(this.frmmain_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trk_motor_position)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_motor_index)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_motor_position)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_set_position)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button btnfind;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Label lab_status;
        private System.Windows.Forms.Button btn_can1send;
        private System.Windows.Forms.Button btn_can1read;
        private System.Windows.Forms.Button set_baud1;
        private System.Windows.Forms.TextBox dataA0;
        private System.Windows.Forms.TextBox dataA1;
        private System.Windows.Forms.TextBox dataA2;
        private System.Windows.Forms.TextBox dataA3;
        private System.Windows.Forms.TextBox dataB0;
        private System.Windows.Forms.TextBox dataB1;
        private System.Windows.Forms.TextBox dataB2;
        private System.Windows.Forms.TextBox dataB3;
        private System.Windows.Forms.Button can2_send;
        private System.Windows.Forms.TextBox dataA10;
        private System.Windows.Forms.TextBox dataA11;
        private System.Windows.Forms.TextBox dataA12;
        private System.Windows.Forms.TextBox dataA13;
        private System.Windows.Forms.TextBox dataB10;
        private System.Windows.Forms.TextBox dataB11;
        private System.Windows.Forms.TextBox dataB12;
        private System.Windows.Forms.TextBox dataB13;
        private System.Windows.Forms.ComboBox cmb_baud;
        private System.Windows.Forms.Button set_baud2;
        private System.Windows.Forms.ComboBox cmb_baud2;
        private System.Windows.Forms.TextBox txt_can1_id;
        private System.Windows.Forms.TextBox txt_can2_id;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btn_text_clear;
        private System.Windows.Forms.Button btn_auto_read;
        private Sunisoft.IrisSkin.SkinEngine skinEngine1;
        private System.Windows.Forms.RichTextBox txt_recieved_data;
        private System.Windows.Forms.Button btn_get_adc;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label lab_adc_disp0;
        private System.Windows.Forms.TrackBar trk_motor_position;
        private System.Windows.Forms.NumericUpDown num_motor_index;
        private System.Windows.Forms.Button btn_stop_all_motors;
        private System.Windows.Forms.NumericUpDown num_motor_position;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Button Btn_set_position;
        private System.Windows.Forms.NumericUpDown num_set_position;
    }
}

