using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
///////////////
using Microsoft.Win32.SafeHandles;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using Microsoft.VisualBasic;
using System.Collections;
using System.Diagnostics;
using System.Timers;
using System.Threading;

using GenericHid;

namespace usbcommand
{
    public partial class frmmain : Form
    {
        #region vars
        private IntPtr deviceNotificationHandle;
        private Boolean exclusiveAccess;
        private FileStream fileStreamDeviceData;
        private SafeFileHandle hidHandle;
        private String hidUsage;
        private Boolean myDeviceDetected;
        private String myDevicePathName;
        private Boolean transferInProgress = false;

        private Debugging MyDebugging = new Debugging(); //  For viewing results of API calls via Debug.Write.
        private DeviceManagement MyDeviceManagement = new DeviceManagement();
        private Hid MyHid = new Hid();
        internal frmmain FrmMy;
        int err_cnt = 0;


        #endregion
        ///  <summary>
        ///  Uses a series of API calls to locate a HID-class device
        ///  by its Vendor ID and Product ID.
        ///  </summary>
        ///          
        ///  <returns>
        ///   True if the device is detected, False if not detected.
        ///  </returns>
        bool buffer_ovf = false;
        
        private Boolean FindTheHid()
        {
            Boolean deviceFound = false;
            String[] devicePathName = new String[128];
            String functionName = "";
            Guid hidGuid = Guid.Empty;
            Int32 memberIndex = 0;
            Int32 myProductID = 0;
            Int32 myVendorID = 0;
            Boolean success = false;
            

            try
            {
                myDeviceDetected = false;
                CloseCommunications();

                //  Get the device's Vendor ID and Product ID from the form's text boxes.

                try
                {
                    myVendorID = Int32.Parse("c251", NumberStyles.AllowHexSpecifier);
                    myProductID = Int32.Parse("1c01", NumberStyles.AllowHexSpecifier);
                }
                catch (Exception ex)
                {

                }
                //  GetVendorAndProductIDsFromTextBoxes(ref myVendorID, ref myProductID);

                //  ***
                //  API function: 'HidD_GetHidGuid

                //  Purpose: Retrieves the interface class GUID for the HID class.

                //  Accepts: 'A System.Guid object for storing the GUID.
                //  ***

                Hid.HidD_GetHidGuid(ref hidGuid);

                functionName = "GetHidGuid";
                Console.WriteLine(MyDebugging.ResultOfAPICall(functionName));
                Console.WriteLine("  GUID for system HIDs: " + hidGuid.ToString());

                //  Fill an array with the device path names of all attached HIDs.

                deviceFound = MyDeviceManagement.FindDeviceFromGuid(hidGuid, ref devicePathName);

                //  If there is at least one HID, attempt to read the Vendor ID and Product ID
                //  of each device until there is a match or all devices have been examined.

                if (deviceFound)
                {
                    memberIndex = 0;

                    do
                    {
                        //  ***
                        //  API function:
                        //  CreateFile

                        //  Purpose:
                        //  Retrieves a handle to a device.

                        //  Accepts:
                        //  A device path name returned by SetupDiGetDeviceInterfaceDetail
                        //  The type of access requested (read/write).
                        //  FILE_SHARE attributes to allow other processes to access the device while this handle is open.
                        //  A Security structure or IntPtr.Zero. 
                        //  A creation disposition value. Use OPEN_EXISTING for devices.
                        //  Flags and attributes for files. Not used for devices.
                        //  Handle to a template file. Not used.

                        //  Returns: a handle without read or write access.
                        //  This enables obtaining information about all HIDs, even system
                        //  keyboards and mice. 
                        //  Separate handles are used for reading and writing.
                        //  ***

                        // Open the handle without read/write access to enable getting information about any HID, even system keyboards and mice.

                        hidHandle = FileIO.CreateFile(devicePathName[memberIndex], 0, FileIO.FILE_SHARE_READ | FileIO.FILE_SHARE_WRITE, IntPtr.Zero, FileIO.OPEN_EXISTING, 0, 0);

                        functionName = "CreateFile";
                        // Debug.WriteLine(MyDebugging.ResultOfAPICall(functionName));

                        // Debug.WriteLine("  Returned handle: " + hidHandle.ToString());

                        if (!hidHandle.IsInvalid)
                        {
                            //  The returned handle is valid, 
                            //  so find out if this is the device we're looking for.

                            //  Set the Size property of DeviceAttributes to the number of bytes in the structure.

                            MyHid.DeviceAttributes.Size = Marshal.SizeOf(MyHid.DeviceAttributes);

                            //  ***
                            //  API function:
                            //  HidD_GetAttributes

                            //  Purpose:
                            //  Retrieves a HIDD_ATTRIBUTES structure containing the Vendor ID, 
                            //  Product ID, and Product Version Number for a device.

                            //  Accepts:
                            //  A handle returned by CreateFile.
                            //  A pointer to receive a HIDD_ATTRIBUTES structure.

                            //  Returns:
                            //  True on success, False on failure.
                            //  ***                            

                            success = Hid.HidD_GetAttributes(hidHandle, ref MyHid.DeviceAttributes);

                            if (success)
                            {
                                Console.WriteLine("  HIDD_ATTRIBUTES structure filled without error.");
                                Console.WriteLine("  Structure size: " + MyHid.DeviceAttributes.Size);
                                Console.WriteLine("  Vendor ID: " + Convert.ToString(MyHid.DeviceAttributes.VendorID, 16));
                                Console.WriteLine("  Product ID: " + Convert.ToString(MyHid.DeviceAttributes.ProductID, 16));
                                Console.WriteLine("  Version Number: " + Convert.ToString(MyHid.DeviceAttributes.VersionNumber, 16));

                                //  Find out if the device matches the one we're looking for.

                                if ((MyHid.DeviceAttributes.VendorID == myVendorID) && (MyHid.DeviceAttributes.ProductID == myProductID))
                                {
                                    Console.WriteLine("  My device detected");

                                    //  Display the information in form's list box.

                                    //        lstResults.Items.Add( "Device detected:" );                                   
                                    //        lstResults.Items.Add("  Vendor ID= " + Convert.ToString(MyHid.DeviceAttributes.VendorID, 16));                                  
                                    //        lstResults.Items.Add("  Product ID = " + Convert.ToString(MyHid.DeviceAttributes.ProductID, 16));

                                    // ScrollToBottomOfListBox();

                                    myDeviceDetected = true;

                                    //  Save the DevicePathName for OnDeviceChange().

                                    myDevicePathName = devicePathName[memberIndex];
                                }
                                else
                                {
                                    //  It's not a match, so close the handle.

                                    myDeviceDetected = false;
                                    hidHandle.Close();
                                }
                            }
                            else
                            {
                                //  There was a problem in retrieving the information.

                                Console.WriteLine("  Error in filling HIDD_ATTRIBUTES structure.");
                                myDeviceDetected = false;
                                hidHandle.Close();
                            }
                        }

                        //  Keep looking until we find the device or there are no devices left to examine.

                        memberIndex = memberIndex + 1;
                    }
                    while (!((myDeviceDetected || (memberIndex == devicePathName.Length))));
                }

                if (myDeviceDetected)
                {
                    //  The device was detected.
                    //  Register to receive notifications if the device is removed or attached.
                    // USB_Status.Text = "USB Status: Attached (VID: " + txtVendorID.Text + " PID: " + txtProductID.Text + ")";

                    success = MyDeviceManagement.RegisterForDeviceNotifications(myDevicePathName, FrmMy.Handle, hidGuid, ref deviceNotificationHandle);

                    Console.WriteLine("RegisterForDeviceNotifications = " + success);

                    //  Learn the capabilities of the device.

                    MyHid.Capabilities = MyHid.GetDeviceCapabilities(hidHandle);

                    if (success)
                    {
                        //  Find out if the device is a system mouse or keyboard.

                        hidUsage = MyHid.GetHidUsage(MyHid.Capabilities);

                        //  Get the Input report buffer size.

                        GetInputReportBufferSize();
                        //  cmdInputReportBufferSize.Enabled = true;

                        //Close the handle and reopen it with read/write access.

                        hidHandle.Close();
                        hidHandle = FileIO.CreateFile(myDevicePathName, FileIO.GENERIC_READ | FileIO.GENERIC_WRITE, FileIO.FILE_SHARE_READ | FileIO.FILE_SHARE_WRITE, IntPtr.Zero, FileIO.OPEN_EXISTING, 0, 0);

                        if (hidHandle.IsInvalid)
                        {
                            exclusiveAccess = true;
                            // lstResults.Items.Add("The device is a system " + hidUsage + ".");
                            //lstResults.Items.Add("Windows 2000 and Windows XP obtain exclusive access to Input and Output reports for this devices.");
                            //lstResults.Items.Add("Applications can access Feature reports only.");
                            //ScrollToBottomOfListBox();
                        }

                        else
                        {
                            if (MyHid.Capabilities.InputReportByteLength > 0)
                            {
                                //  Set the size of the Input report buffer. 

                                Byte[] inputReportBuffer = null;

                                inputReportBuffer = new Byte[MyHid.Capabilities.InputReportByteLength];

                                fileStreamDeviceData = new FileStream(hidHandle, FileAccess.Read | FileAccess.Write, inputReportBuffer.Length, false);
                            }

                            if (MyHid.Capabilities.OutputReportByteLength > 0)
                            {
                                Byte[] outputReportBuffer = null;
                                outputReportBuffer = new Byte[MyHid.Capabilities.OutputReportByteLength];
                            }

                            //  Flush any waiting reports in the input buffer. (optional)

                            MyHid.FlushQueue(hidHandle);
                        }
                    }
                }
                else
                {
                    //  The device wasn't detected.
                    //USB_Status.Text = "USB Status : Device not found";
                    //lstResults.Items.Add("Device not found.");
                    //cmdInputReportBufferSize.Enabled = false;
                    //cmdOnce.Enabled = true;

                    Console.WriteLine(" Device not found.");
                    return false;
                    //ScrollToBottomOfListBox();
                }
                return myDeviceDetected;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);

                throw;
            }
        }

        /// <summary>
        /// Close the handle and FileStreams for a device.
        /// </summary>
        /// 
        private void CloseCommunications()
        {
            if (fileStreamDeviceData != null)
            {
                fileStreamDeviceData.Close();
            }

            if ((hidHandle != null) && (!(hidHandle.IsInvalid)))
            {
                hidHandle.Close();
            }

            // The next attempt to communicate will get new handles and FileStreams.

            myDeviceDetected = false;
        }

        ///  <summary>
        ///  Finds and displays the number of Input buffers
        ///  (the number of Input reports the host will store). 
        ///  </summary>

        private void GetInputReportBufferSize()
        {
            Int32 numberOfInputBuffers = 0;
            Boolean success;

            try
            {
                //  Get the number of input buffers.

                success = MyHid.GetNumberOfInputBuffers(hidHandle, ref numberOfInputBuffers);

                //  Display the result in the text box.

                //txtInputReportBufferSize.Text = Convert.ToString(numberOfInputBuffers);
            }
            catch (Exception)
            {
                // DisplayException(this.Name, ex);
                //throw;
            }
        }

        
        /// //////////////////////////////////////////////////
        private int read_char(ref Byte buff)
        {
            int index = 0;
        
            Byte[] inputReportBuffer;
            try
            {
                send_string("a");
                inputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.InputReportByteLength < 0) { Console.WriteLine("read error"); return 0; }
                inputReportBuffer = new Byte[MyHid.Capabilities.InputReportByteLength];
               
                
                 
                MyHid.GetInputReportViaControlTransfer(hidHandle, ref inputReportBuffer);
                if (inputReportBuffer[1] == 0) return 0;
                buff=inputReportBuffer[1];
                
                
                return 1;
            }
            catch (Exception ex) {
               Console.WriteLine(ex.Message);
                return 0;
            }
        }
        private int send_string(string data)
        {
            try
            {
                Byte[] outputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.OutputReportByteLength < 0) return 0;
                outputReportBuffer = new Byte[MyHid.Capabilities.OutputReportByteLength];
                outputReportBuffer[0] = 0;
                for (int i = 0; i < data.Length; i++)
                {
                    outputReportBuffer[0] = 0;
                     outputReportBuffer[1] = (byte)data[i];
                    MyHid.SendOutputReportViaControlTransfer(hidHandle, outputReportBuffer);

                }
                return 1;
            }
            catch (Exception ex) {
               
                Console.WriteLine(ex.Message);
                return 0;
            }
        }

        private int hid_write(byte []data)
        {
            try
            {
                Byte[] outputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.OutputReportByteLength < 0) { Console.WriteLine("size error"); return 0; }
                outputReportBuffer = new Byte[MyHid.Capabilities.OutputReportByteLength];
                outputReportBuffer[0] = 0xaa;
             
                   
                    MyHid.SendOutputReportViaControlTransfer(hidHandle, data);

                return 1;
            }
            catch (Exception ex)
            {

                Console.WriteLine(ex.Message);
                return 0;
            }
        }

        public frmmain()
        {
            InitializeComponent();
            skinEngine1.SkinFile = "PageColor2.ssk";// comboBox1.SelectedItem.ToString();
            //skinEngine1.SkinFile = "Wave.ssk";
            skinEngine1.Active = true;
        }

        private void btnfind_Click(object sender, EventArgs e)
        {

            {
                if (!FindTheHid()) {
                    lab_status.Text=("divice doesnt exist!");
                    return;
                   
                }
                lab_status.Text= "connected!";
            }

        }

        private void frmmain_Load(object sender, EventArgs e)
        {
            FrmMy = this;
        }

        private void btnrw_Click(object sender, EventArgs e)
        {
            //ExchangeInputAndOutputReports();
            send_string("amin\n");
           

        }

        private bool chk_parity(byte[] data,int start_index,int count) {
            byte result=0;
            for (int i = start_index   ; i < start_index+count; i++)
            {
                result^=data[i];
            }
            if (result == 255)return true;
                
         return false;
        }
        int pos=0;
        private void timer1_Tick(object sender, EventArgs e)
        {
            Byte[] inputReportBuffer;
            try
            {

                inputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.InputReportByteLength < 0) { MessageBox.Show("error"); Console.WriteLine("read error"); return; }
                inputReportBuffer = new Byte[MyHid.Capabilities.InputReportByteLength];



                MyHid.GetInputReportViaControlTransfer(hidHandle, ref inputReportBuffer);
               
                if (inputReportBuffer[22] == 0 && inputReportBuffer[21] == 0) return;
                txt_recieved_data.SelectionStart = txt_recieved_data.Text.Length;
                txt_recieved_data.ScrollToCaret();
                for (int i = 0; i < 20; i++)
                {
                   // txt_recieved_data.Text = txt_recieved_data.Text + inputReportBuffer[i + 1].ToString("x") + " ";
                    txt_recieved_data.AppendText(inputReportBuffer[i + 1].ToString("x") + " ");
                }

                txt_recieved_data.AppendText("\n");
            }
            catch (Exception)
            {

            }



        }
        public static string ByteArrayToStr(byte[] byteArray)
        {
            System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
            return encoding.GetString(byteArray);
        }
       // ####################################################String to byte array:
public static byte[] StrToByteArray(string str)
{
    System.Text.ASCIIEncoding  encoding = new System.Text.ASCIIEncoding();
    return encoding.GetBytes(str);
}
        private string read_string() {
            try
            {
                byte dt = 0;
                int index = 0;
                byte[] buff = new byte[100];

            itter:
                dt = 0;
                read_char(ref dt);
                buff[index++] = dt;
                if (dt != 0 && index <100) goto itter;
                return ByteArrayToStr(buff);
            }
            catch (Exception ex) { MessageBox.Show(ex.Message);
            return " ";
            }

        }
        private void button1_Click(object sender, EventArgs e)
        {
         
            try
            {
               Console.WriteLine(read_string());
            
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        private void btn_send_file_Click(object sender, EventArgs e)
        {
            byte[] buff;
            buff=new byte[16];
            buff[6] = 15;//command=spi read
            buff[10] = 25;//arg=data count
            hid_write(buff);//call set out report
        }
        unsafe private float get_float_from_array(byte[] inp, int index)
        {
            float result=0;
            byte* pointer;

            pointer = (byte*)&(result);
            *(pointer + 0) = inp[index+3];
            *(pointer + 1) = inp[index+2];
            *(pointer + 2) = inp[index+1];
            *(pointer + 3) = inp[index];
          
   
            return result;

        }

        unsafe private Int32 get_int_from_array(byte[] inp,int index ) {

            
       Int32 result=0;
       result += inp[index+1];
       result = result << 8;
       result += inp[index+2];
       result = result << 8;
       result += inp[index+3];
       result = result << 8;
       result += inp[index+4];
       return result;
        
        }

        private void btn_auto_read_Click(object sender, EventArgs e)
        {
            timer1.Enabled = true;
        }
        unsafe private void floa_to_array(float inp,ref byte[]output){

            byte* point;
            point = (byte*)&(inp);

            output[0] = *point;
            output[1] = *(point+1);
            output[2] = *(point + 2);
            output[3] = *(point + 3);

        }


        unsafe private void floak_to_array(float inp, ref byte[] output)
        {

            byte* point;
            point = (byte*)&(inp);

            output[0] = *point;
            output[1] = *(point + 1);

        }

        private void btn_can1send_Click(object sender, EventArgs e)
        {


            int j;
            byte[] buffer;
            byte[] input;
            input = new byte[65];
            input[0] = byte.Parse(dataA0.Text,NumberStyles.HexNumber);
            input[1] = byte.Parse(dataA1.Text,NumberStyles.HexNumber);
            input[2] = byte.Parse(dataA2.Text,NumberStyles.HexNumber);
            input[3] = byte.Parse(dataA3.Text,NumberStyles.HexNumber);
            input[4] = byte.Parse(dataB0.Text,NumberStyles.HexNumber);
            input[5] = byte.Parse(dataB1.Text,NumberStyles.HexNumber);
            input[6] = byte.Parse(dataB2.Text,NumberStyles.HexNumber);
            input[7] = byte.Parse(dataB3.Text,NumberStyles.HexNumber);
            
            buffer = new byte[65];
            buffer[1] = 1;
            buffer[2] =(byte) (int.Parse(txt_can1_id.Text,NumberStyles.HexNumber)>>8);
            buffer[3] = (byte)(int.Parse(txt_can1_id.Text, NumberStyles.HexNumber) & 0xff);
            for (j = 0; j < 8; j++)
            {

                buffer[j + 4] = input[j];
            }

            hid_write(buffer);


        }

        private void btn_can1read_Click(object sender, EventArgs e)
        {
            
            Byte[] inputReportBuffer;
            try
            {

                inputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.InputReportByteLength < 0) { MessageBox.Show("error"); Console.WriteLine("read error"); return; }
                inputReportBuffer = new Byte[MyHid.Capabilities.InputReportByteLength];



                MyHid.GetInputReportViaControlTransfer(hidHandle, ref inputReportBuffer);
                if (inputReportBuffer[22] == 0 && inputReportBuffer[21] == 0) return;
                for (int i = 0; i < 20; i++)
                {
                    txt_recieved_data.Text = txt_recieved_data.Text + inputReportBuffer[i + 1].ToString("x") + " ";
                }
                txt_recieved_data.Text += "\r\n";
              
            }
          
            
            catch(Exception)
            {

            }

        }
       
        private void button2_Click(object sender, EventArgs e)
        {
            byte[] buffer;
            buffer = new byte[65];
            int input = int.Parse(cmb_baud.Text);
            buffer[1] = 4;
            buffer[2] = (byte)(input >> 24);
            buffer[3] = (byte)(input >> 16);
            buffer[4] = (byte)(input >> 8);
            buffer[5] = (byte)(input);
            hid_write(buffer);
        }

        private void can2_send_Click(object sender, EventArgs e)
        {
            int j;
            byte[] buffer;
            byte[] input;
            input = new byte[65];
            input[0] = byte.Parse(dataA10.Text, NumberStyles.HexNumber);
            input[1] = byte.Parse(dataA11.Text, NumberStyles.HexNumber);
            input[2] = byte.Parse(dataA12.Text, NumberStyles.HexNumber);
            input[3] = byte.Parse(dataA13.Text, NumberStyles.HexNumber);
            input[4] = byte.Parse(dataB10.Text, NumberStyles.HexNumber);
            input[5] = byte.Parse(dataB11.Text, NumberStyles.HexNumber);
            input[6] = byte.Parse(dataB12.Text, NumberStyles.HexNumber);
            input[7] = byte.Parse(dataB13.Text, NumberStyles.HexNumber);

            buffer = new byte[65];
            buffer[1] = 2;
            buffer[2] = (byte)(int.Parse(txt_can2_id.Text, NumberStyles.HexNumber) >> 8);
            buffer[3] = (byte)(int.Parse(txt_can2_id.Text, NumberStyles.HexNumber) & 0xff);
            for (j = 0; j < 8; j++)
            {

                buffer[j + 4] = input[j];
            }

            hid_write(buffer);
        }
        void can_read() {
            byte[] buf;
            buf = new byte[16];

            Byte[] inputReportBuffer;
            try
            {

                inputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.InputReportByteLength < 0) { MessageBox.Show("error"); Console.WriteLine("read error"); return; }
                inputReportBuffer = new Byte[MyHid.Capabilities.InputReportByteLength];



                MyHid.GetInputReportViaControlTransfer(hidHandle, ref inputReportBuffer);
                if (inputReportBuffer[22] == 0) return;
                if (inputReportBuffer[11] == 1 & inputReportBuffer[12] == 0x81 && inputReportBuffer[13] == 0x0e) { buffer_ovf = true; }
                if (inputReportBuffer[11] == 1 & inputReportBuffer[12] == 0x81 && inputReportBuffer[13] == 0x0c) { buffer_ovf = false; }
               
            }
            catch (Exception)
            {

            }
        }
        void add_to_fifo( Int32 position, Int32 velocity, int time){
            byte[] buffer;

            buffer = new byte[16];


            buffer[1] = 0x02;
            buffer[2] = 0x05;
            buffer[3] = 0x01;
            buffer[4] = (byte)(position & 0xff);
            buffer[5] = (byte)((position >> 8) & 0xff);
            buffer[6] = (byte)((position >> 16) & 0xff);
            buffer[7] = (byte)((position >> 24) & 0xff);
            buffer[8] = (byte)(velocity & 0xff);
            buffer[9] = (byte)((velocity >> 8) & 0xff);
            buffer[10] = (byte)((velocity >> 24) & 0xff);
            buffer[11] = (byte)(time & 0xff);
            hid_write(buffer);

}
        void start_ipm_trajectory()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 0x02;
            buffer[2] = 0x02;
            buffer[3] = 0x01;
            buffer[4] = 0x1f;
            hid_write(buffer);

        }
        void reset_all_node() {
            byte[] buffer;
            buffer = new byte[16];
           
            for (int i = 0; i < 16; i++)
			{
                buffer[i] = 0;
			}
            buffer[1] = 2;
            buffer[4] = 0x81;
            hid_write(buffer);
        
        }
        void set_preoperational()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 2;
            buffer[4] = 0x80;
            hid_write(buffer);

        }
        void start_node()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 2;
            buffer[4] = 0x1;
            hid_write(buffer);

        }
        void set_ipm_mode()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 2;
            buffer[2] = 0x03;
            buffer[3] = 0x01;
            buffer[4] = 0x07;
            hid_write(buffer);

        }
        void switch_on()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 2;
            buffer[2] = 0x02;
            buffer[3] = 0x01;
            buffer[4] = 0x0f;
            hid_write(buffer);

        }
        void switch_off()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 2;
            buffer[2] = 0x02;
            buffer[3] = 0x01;
            buffer[4] = 0x06;
            hid_write(buffer);

        }
        void clear_fifo()
        {
            byte[] buffer;
            buffer = new byte[16];

            for (int i = 0; i < 16; i++)
            {
                buffer[i] = 0;
            }
            buffer[1] = 2;
            buffer[2] = 0x04;
            buffer[3] = 0x01;
            buffer[4] = 0x01;
            hid_write(buffer);

        }
        
        private void set_baud2_Click(object sender, EventArgs e)
        {
            byte[] buffer;
            buffer = new byte[65];
            int input = int.Parse(cmb_baud2.Text);
            buffer[1] = 5;
            buffer[2] = (byte)(input >> 24);
            buffer[3] = (byte)(input >> 16);
            buffer[4] = (byte)(input >> 8);
            buffer[5] = (byte)(input);
            hid_write(buffer);
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
        
        }

        private void btn_text_clear_Click(object sender, EventArgs e)
        {
            txt_recieved_data.Clear();
        }

        private void btn_auto_read_Click_1(object sender, EventArgs e)
        {

           if (btn_auto_read.Text == "Auto Read OFF")
                {

                    btn_auto_read.Text = "Auto Read ON";
                    timer1.Enabled = true;
                }
                else
                {
                    btn_auto_read.Text = "Auto Read OFF";
                    timer1.Enabled = false;

                }
           
        }
        private bool can_read(ref Byte[] inputReportBuffer)
        {
           // Byte[] inputReportBuffer;
            try
            {

                inputReportBuffer = null;
                if (hidHandle.IsInvalid || MyHid.Capabilities.InputReportByteLength < 0) { MessageBox.Show("error"); Console.WriteLine("read error"); return false; }
                inputReportBuffer = new Byte[MyHid.Capabilities.InputReportByteLength];



                MyHid.GetInputReportViaControlTransfer(hidHandle, ref inputReportBuffer);

                return true;
            }
            catch (Exception)
            {
                return false;
            }



        }
        private void can_send(UInt32 id, byte[] input)
        {
            int j;
            byte[] buffer;
            buffer = new byte[65];
            buffer[1] = 1;
            buffer[2] = (byte)(id >> 8);
            buffer[3] = (byte)(id& 0xff);
            for (j = 0; j < 8; j++)
            {

                buffer[j + 4] = input[j];
            }

            hid_write(buffer);
        }
        private void btn_stop_all_motors_Click(object sender, EventArgs e)
        {
            byte[]  input = new byte[65];
            input[0] = 4;
            input[1] = (byte)Convert.ToInt32(num_motor_index.Value);

            can_send((UInt32)Num_can_id.Value, input);
        }

        private void trk_motor_position_Scroll(object sender, EventArgs e)
        {
            byte[] input = new byte[65];



            input[0] = 3;
            input[1] = (byte)Convert.ToInt32(num_motor_index.Value);
            input[2] = (byte)(trk_motor_position.Value & 0xff);
            input[3] = (byte)((trk_motor_position.Value>>  8) & 0xff);
            input[4] = (byte)((trk_motor_position.Value >> 16) & 0xff);
            input[5] = (byte)((trk_motor_position.Value >> 24) & 0xff);


            can_send((UInt32)Num_can_id.Value, input);
            num_motor_position.Value = trk_motor_position.Value;

        }
        Int32 read_adc_channel(byte channel)
        {
            Int32 adc =0;
            
            byte[] tx_data = new byte[65];
            Byte[] rx_data = new Byte[65];
            tx_data[0] = 2;
            tx_data[1] = channel;

            can_send((UInt32)Num_can_id.Value, tx_data);
            Thread.Sleep(10);
            for (int i = 0; i < 200; i++)
            {
                if (can_read(ref rx_data)) {
                    adc = rx_data[3];
                    adc <<= 8;
                    adc |= rx_data[4];

                    return adc;
                }

            }

          
            return -1;


        }
        float read_bmp_channel(byte channel)
        {
            float bmp = 0;

            byte[] tx_data = new byte[65];
            Byte[] rx_data = new Byte[65];
            tx_data[0] = 5;
            tx_data[1] = channel;

            can_send((UInt32)Num_can_id.Value, tx_data);
            Thread.Sleep(10);
            for (int i = 0; i < 200; i++)
            {
                if (can_read(ref rx_data))
                {

                    float value = BitConverter.ToSingle(rx_data, 3);
                    return value;
                }
            }


            return -1;


        }
        Int32 set_position(UInt16 value)
        {
         

            byte[] tx_data = new byte[65];
            tx_data[0] = 0;
            tx_data[1] = (byte)Convert.ToInt32(num_motor_index.Value);
            tx_data[2] = (byte)((int)value & 0xff);
            tx_data[3] = (byte)(((int)value >> 8) & 0xff);
            tx_data[4] = (byte)(((int)value >> 16) & 0xff);
            tx_data[5] = (byte)(((int)value >> 24) & 0xff);
            can_send((UInt32)Num_can_id.Value, tx_data);

            return 0;
        }
        private void btn_get_adc_Click(object sender, EventArgs e)
        {
      
            lab_adc_disp0.Text = read_adc_channel((byte)0).ToString();
            lab_adc_disp0.Text += " "+ read_adc_channel((byte)1).ToString();
            lab_adc_disp0.Text += " " + read_adc_channel((byte)2).ToString();
            lab_adc_disp0.Text += " " + read_adc_channel((byte)3).ToString();
            lab_adc_disp0.Text += " " + read_adc_channel((byte)4).ToString();
            lab_adc_disp0.Text += " " + read_adc_channel((byte)5).ToString();

        }

        private void num_motor_position_ValueChanged(object sender, EventArgs e)
        {
            trk_motor_position.Value = (int)num_motor_position.Value;
        }

        private void Btn_set_position_Click(object sender, EventArgs e)
        {
            set_position((UInt16)num_set_position.Value);
        }

        private void btn_read_pressure_Click(object sender, EventArgs e)
        {

            lab_pressure_values.Text = read_bmp_channel((byte)0).ToString();
            lab_pressure_values.Text += " " + read_bmp_channel((byte)1).ToString();
            lab_pressure_values.Text += " " + read_bmp_channel((byte)2).ToString();
            lab_pressure_values.Text += " " + read_bmp_channel((byte)3).ToString();
            lab_pressure_values.Text += " " + read_bmp_channel((byte)4).ToString();
            lab_pressure_values.Text += " " + read_bmp_channel((byte)5).ToString();
        }

        private void Num_can_id_ValueChanged(object sender, EventArgs e)
        {

        }
    }
}
