======================== README.TXT =================================
ARTICLE INFORMATION

   All Authors:
   Isaiah Morgenstern, Shan Zhong, Qimin Zhang, Logan Baker, 
   Jeremy Norris, Bao Tran, and Arne Schwettmann

   Title:
   FPGA-Controlled Versatile Microwave Source for Cold Atom Experiments

DEPOSIT INFORMATION

   Description:
   This archive contains the source codes and files of our FPGA-based
   microwave source (dds_driver). dds_driver is written in Verilog and is intended
   to be uploaded to a DE0-CV FPGA board. The compiler used was Quartus II 14.0.

   Total No. of Files:
   44

   File Names:
   (\)
   "README.TXT"
      - this file
   "Pin_Assignments.pdf"
      - Describes how to connect all boards to FPGA device
   (\dds_driver_commercial\)
   "dds_driver.qpf"
	- Quartus project file, open this file in Quartus II to
	  access all other files for modification
   "dds_driver.v"
	- top level source code, main FPGA program
   "usb_registers.v"
	- usb communication submodule
   "manual_freq.v"
	- manual frequency control submodule
   "manual_amp.v"
        - manual amplitude control submodule
   "lcd_write.v"
	- lcd control submodule
   "dds_write.v"
	- dds control submodule
   "dds_driver.pof"
	- Quartus file to be uploaded to FPGA
   **Other Verilog files are for compiling modules in Quartus II 14.0 but aren't interacted with directly**
   **Verilog files in (/db/) are from compiling the code.**
   **Verilog files in (/incremental_db/) are from past compilations of the code, so you can revert to old
     code versions.**
   (\LabVIEW_DDS_Code\)
   "FT_Get_Device_Description_By_Index.vi"
	- LabVIEW program that finds name of USB UART device to be used in other LabVIEW programs
   "Example_GUI.vi"
	- LabVIEW program that is an example of how to connect to and interact with FPGA/DDS system
	  to produce signals
   **Other LabVIEW files in (/DDS9954/) are called upon by the "Example_GUI.vi" but are not
     interacted with directly, this is where a majority of the files reside**

   File Types:
   *.qpf: Quartus II project file
   *.v:   Quartus II file for designing code to run on FPGA
   *.pof: Quartus II programming file for FPGA
   *.vi:  LabVIEW file for remotely controlling FPGA/DDS

   Instructions: 
   
   Verilog
   The dds_driver.qpf file can be opened in Quartus II and will provide 
   the hierarchy of the setup. From here each file can be accessed and 
   modified individually. The whole program can also be compiled and
   uploaded to the FPGA device using this file. This file is prepared
   for an FPGA device that utilizes a Cyclone V chip. The chip type 
   can be set using this file and Quartus II, note that setting the
   chip type will erase all pin assignments and they will have to be
   reassigned in accordance with the new chip.

   Under Quartus II select the programmer icon and a window will pop
   up, under this window set the mode to "Active Serial Programming"
   and add device EPCS64. Now change file to the dds_driver.pof and
   ensure the FPGA is connected and is set to programming mode.
   Then start the download to get a fully operational FPGA.

   When interfacing your FPGA device with LabVIEW you will have to
   search for connected devices and the FPGA should appear as FT232
   USB UART, this is the USB chip that communicates between the 
   computer and the FPGA device.

   LabVIEW
   In order to interface a LabVIEW program with your FPGA use the
   following steps:
   1) Run the "FT_Get_Device_Description_By_Index.vi" program to get
      the proper name of your device.
   2) Enter the name of your device into the "Microwave_Example_GUI.vi"
      program.
   3) In the "Microwave_Example_GUI.vi" program select "Single Frequency"
      mode and set the start values: frequency to 30 MHz and power to 30 dBm,
      ignore the ramp points and stop values.
   4) Run the LabVIEW program, if your DDS has an external clock at 400 MHz,
      then you should get an output with the characteristics you input.
   **There is also a "Linear Ramp" mode in this program which takes your
     start/stop values and ramp points and creates a table of linearly increasing
     values. The FPGA/DDS system will step to the next value every time it
     recieves a TTL pulse. To generate TTL pulses you can use Key3 on the DE0-CV
     or input a TTL source.
   **These LabVIEW programs are just samples to give you an idea of how to create
     your own LabVIEW program that will run complex sequences of your design. 

   FPGA
   Use the buttons and switches on the FPGA board to test the device in manual
   mode. Below is the layout of this buttons/switches and their functionallity:
   
   Key0 -> Decrease Value of selected parameter 
   Key1 -> Increase Value of selected parameter
   Key2 -> Switch between parameters
   Key3 -> TTL signal (Note it is better to use a real TTL input as this is not
           as reliable as an actual TTL input. This is for easy testing in 
           conjunction with the sample LabVIEW code)

   Switch0 -> Change between remote and manual mode
   Switch2 -> Flip up and then down to reinitialize the LCD screen

   LEDR0 -> Is on when in remote mode.
   LEDR8 -> Changes status with each TTL pulse, i.e. if off will turn on when
            receiving TTL pulse and remain on until recieving next pulse.
   LEDR9 -> Last bit value when recieving USB data. (Mainly and indicator that
            the FPGA is receiving data via USB)

   LCD
   Make sure the LCD is in spi mode or it will not operate properly. See datasheet
   for how to set mode on LCD.

   DDS
   On the DDS there is a selector to change between the external and internal clock
   make sure it is on the external clock.

   Download Links:
   Quartus II 14.0 can be downloaded from Intel at the URL
   https://fpgasoftware.intel.com/14.0/ (retrieved August
   2018).

   LabVIEW can be downloaded from National Instruments at the URL
   http://www.ni.com/en-us/shop/labview/download.html
   (retrieved August 2018).

   Changelog: v3.79 (First Release)

   Contact Information:
   Arne Schwettmann
   The University of Oklahoma
   Homer L. Dodge Department of Physics and Astronomy
   440 W. Brooks St.
   Norman, OK 73019
   e-mail: schwettmann@ou.edu

 =====================================================================
