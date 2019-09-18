// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group

// Module Name:
// dds_driver

// Project Name:
// dds_driver

// Target Devices:
// DE0-CV Cyclone V FPGA board


module dds_driver(clk,
				  LED,
				  usbtalk,
				  TTLLED,
				  TTLbutton,
				  in2,
				  ext,
				  out1,
				  out3,
				  switch,
				  FPGA_switch0,
				  FPGA_button0,
				  FPGA_button1,
				  FPGA_button2,
				  usbtxd,
				  adclk,
				  adcs,
				  addin,
				  addout);

input			clk;
input			in2;
input			TTLbutton; 				// Allows user to trigger TTL pulse using KEY0 on FPGA
input			switch;
input			FPGA_switch0; 			// Control remoteEnable from FPGA board
input			FPGA_button0; 			// Control value down from FPGA board
input			FPGA_button1; 			// Control value up from FPGA board
input			FPGA_button2; 			// Control menu mode from FPGA board
input			usbtxd;					// serial input from serial-to-usb converter (note: txd because TRANSMITTED by usb, but rxd for module because RECEIVED by fpga)
input			addout;					// serial data from/out-of the ADC

output	[2:0]	ext;
output	[1:0]	out1;
output	[7:0]	out3;
output			LED;
output			usbtalk; 				// lights up when usb is sending data
output			TTLLED;  				// lights up when recieving TTL signal
output			adclk;
output			adcs;
output			addin;					// serial data into the ADC

// Misc. registers
reg		[31:0]	counter;
reg		[31:0]	waitCnt;
reg					remoteEnable;		// controlled by external switch; high = remote (computer table), low = manual (buttons)
reg					dispSelTrig;		// controlled by external push-button to execute diplay change
reg					downTrig;			// controlled by external push-button to execute down increment
reg					upTrig;				// controlled by external push-button to execute up increment

// DDS control registers
reg		[31:0]	cfr1;					// CFR1 (control function register) to be sent to the DDS
reg		[23:0]	cfr2;					// CFR2 (control function register) to be sent to the DDS
reg		[13:0]	asf;					// ASF (amp scale factor) to be sent to the DDS
reg		[31:0]	ftw0;					// FTW0 (freq tuning word) to be sent to the DDS
reg		[13:0]	pow;					// POW (phase offset word) to be sent to the DDS
reg		[31:0]	ftw1;					// FTW1 (freq tuning word) to be sent to the DDS (used only in linear sweep mode)
reg		[39:0]	nlscw;				// NLSCW (negative linear sweep control word) to be sent to the DDS (used only in linear sweep mode)
reg		[39:0]	plscw;				// PLSCW (positive linear sweep control word) to be sent to the DDS (used only in linear sweep mode)
reg					ps0;					// PS0 (profile select 0); in linear sweep mode, this determines the sweep direction
reg		[31:0]	prvcfr1;				// previous CFR1
reg		[23:0]	prvcfr2;				// previous CFR2
reg		[13:0]	prvasf;				// previous ASF
reg		[31:0]	prvftw0;				// previous FTW0
reg		[13:0]	prvpow;				// previous POW
reg		[31:0]	prvftw1;				// previous FTW1
reg		[39:0]	prvnlscw;			// previous NLSCW
reg		[39:0]	prvplscw;			// previous PLSCW
reg		[8:0]		ddsFlags;			// flags indicate new data to be written to the DDS
reg					ddsWriteStatus;	// or'd ddsFlags[8:0], which indicates if any registers are still waiting to be updated
reg		[4:0]		ddsSend;				// controls writing to DDS; ddsSend[4] high = begin write operation; ddsSend[3:0] is hex register address (0-2,4-8) in DDS
reg					ddsUpdate;			// indicates if/when a DDS ioupdate is needed

// DDS initialization registers
reg					ddsInit;				// indicates if DDS initialization has been completed
reg		[2:0]		ddsInitStep;
reg		[31:0]	ddsWaitCnt;
reg		[31:0]	cfr1Init;
reg		[23:0]	cfr2Init;

// DDS wires/pins
wire				ddscs;					// DDS chip-select pin (when low, indicates write is in progress
wire				ddssclk;					// DDS serial clk pin
wire				ddssdio;					// DDS sdio pin
reg				ddsioupdate;			// DDS ioupdate pin (when high, register buffers are written to the DDS core)
reg				ddsps0;					// DDS PS0 (profile select pin 0); in linear sweep mode, this determines the sweep direction
reg				ddsps1;					// DDS PS1 (profile select pin 1); not currently used (so gnd)
reg				ddsosk;					// DDS OSK (offset keying pin); currently we only do OSK (amplitude control) "manually", so gnd it
reg				ddsrst;					// DDS reset pin

// LCD registers
reg		[2:0]		lcdSend;
reg		[7:0]		lcdData;
reg		[3:0]		lcdAction;
reg					lcdWelcome;
reg		[3:0]		lcdWStep;
reg		[1:0]		lcdStep;
reg		[8:0]		charCnt;
reg		[127:0]	lcdMessL1;			// line 1 of message
reg		[3:0]		lcdDisplay;			// current display/manual control; 0: increment/decrement frequency in MHz; 1: inc/dec freq in kHz; 2: inc/dec amp in %;
reg					lcdDispStep;		// steps in setting which status to display
reg		[25:0]	dispSetWaitCnt;	// counts wait time between setting which status message to display
reg		[1:0]		dispSetStep;		// status of state machine setting which status message to display
reg					resetStep;

// LCD wires/pins
wire					lcdcsb;
wire					lcdscl;
wire					lcdsi;
wire		[23:0]	freqBCD;				// the freq of the DDS in BCD (up to 200.000 MHz)
wire		[11:0]	ampBCD;				// the amp of the DDS in BCD (up to 100%)

// DDS wires/pins
wire		[31:0]	freqMan;				// FTW set by manual control panel
wire		[13:0]	ampMan;				// ASF set by manual control panel

// PLL wires
wire					sclk;					// "slow clk" operates at 20 MHz
wire					ssclk;				// "super slow clk" operates at 10 MHz
wire					fclk;					// "fast clk" operates at 100 MHz
wire					pllLocked;			// status of PLL (1 = locked)

// USB communication module wires
wire		[1:0]		rcontrol;			// control word that determines action upon receiving an external TTL trigger (e.g. step through whole table, or just next line; locked or not)
wire		[31:0]	rcfr1;				// remote CFR1 (single register per cycle)
wire		[23:0]	rcfr2;				// remote CFR2 (single register per cycle)
wire		[13:0]	rasf;					// ASF for FIFO; serves as PI controller set-point in locked-mode
wire		[31:0]	rftw0;				// FTW0 for FIFO
wire		[13:0]	rpow;					// POW for FIFO
wire		[31:0]	rftw1;				// FTW1 for FIFO
wire		[39:0]	rlscw;				// LSCW for FIFO (for both PLSCW and NLSCW)
wire					rps0;					// PS0 for FIFO
wire		[31:0]	rtime;				// TIME for FIFO
wire		[9:0]		rlines;				// number of items to write to the FIFO (lines in the table); max is 1024 (note that sweep-relevant FIFOs depth just 512).
wire		[6:0]		writeReq;			// To input port of FIFO megafunction; bits controlled by submodule.
wire		[6:0]		sclr;					// To input port of FIFO megafunction; bits controlled by submodule.
wire					softTrig;			// Software trigger for DDS (remote, table, no amp lock mode).

// FIFO wires
wire		[9:0]		depth0;				// indicates number of words used in FIFO
wire		[9:0]		depth1;				// indicates number of words used in FIFO
wire		[9:0]		depth2;				// indicates number of words used in FIFO
wire		[7:0]		depth3;				// indicates number of words used in FIFO
wire		[7:0]		depth4;				// indicates number of words used in FIFO
wire		[7:0]		depth5;				// indicates number of words used in FIFO
wire		[8:0]		depth6;				// indicates number of words used in FIFO
wire		[6:0]		empty;				// high bit means that FIFO is empty (each bit represents one FIFO)
wire		[6:0]		full;					// high bit means that FIFO is full (each bit represents one FIFO)
wire		[13:0]	qasf;					// queued ASF from FIFO
wire		[31:0]	qftw0;				// queued FTW0 from FIFO
wire		[13:0]	qpow;					// queued POW from FIFO
wire		[31:0]	qftw1;				// queued FTW1 from FIFO
wire		[39:0]	qlscw;				// queued LSCW from FIFO (for both NLSCW and PLSCW)
wire		[31:0]	qtime;				// queued TIME from FIFO
wire					qps0;					// queued PS0 from FIFO

// FIFO registers
reg		[6:0]		readReq;				// each bit to an input port of a FIFO megafunction; high bit triggers a read from that FIFO
reg		[13:0]	nasf;					// new ASF from FIFO
reg		[31:0]	nftw0;				// new FTW0 from FIFO
reg		[13:0]	npow;					// new POW from FIFO
reg		[31:0]	nftw1;				// new FTW1 from FIFO
reg		[39:0]	nnlscw;				// new NLSCW from FIFO (or auto fast)
reg		[39:0]	nplscw;				// new PLSCW from FIFO (or auto fast)
reg		[31:0]	ntime;				// new TIME from FIFO
reg					ips0;					// intermediate PS0 (used in table-unlocked mode)
reg					dps0;					// delayed PS0 assignment
reg					nps0;					// new PS0 from FIFO

// External trigger registers
reg					triggerA;			// toggles on the positive edge of in2
reg		[2:0]		prevtrigger;		// records previous values of triggerA
reg					extTrig;				// clk-synchronized pulse flag indicating an external trigger was received.

// Software trigger registers
reg					sTrigStep;			// status of software trigger delay state machine.
reg		[15:0]	sTrigWait;			// wait counter for software trigger.
reg					softTrig0;			// delayed software trigger.
reg					firstSwitch;		// Used to clear DDS when switching to remote mode.

// Table/Sequence registers
reg		[1:0]		tableStep;
reg		[2:0]		seqStep;
reg		[1:0]		lkTableStep;
reg		[2:0]		lkSeqStep;
reg		[1:0]		syncStep;
reg		[3:0]		syncWait;
reg		[3:0]		syncPause;			// sets how long to hold IO-UPDATE pulse high; longer during initialization, shorter after DDS internal clk mult running.
reg					getNextLine;
reg		[31:0]	rfTimer;

// Message parameters
parameter	welcomeL1	= {8'h44,8'h44,8'h53,8'hB0,8'h64,8'h72,8'h69,8'h76,8'h65,8'h72,8'h20,8'h76,8'h33,8'h2E,8'h37,8'h39};	// 16 char, 128 bits; line 1 of welcome message

parameter	freqL1		= {8'h46,8'h72,8'h65,8'h71,8'h3D,8'h20,8'h30,8'h30,8'h30,8'h2E,8'h30,8'h30,8'h30,8'h4D,8'h48,8'h7A};	// 16 char, 128 bits; line 1 of frequency message

parameter	ampL1			= {8'h41,8'h6D,8'h70,8'h3D,8'h20,8'h20,8'h30,8'h30,8'h30,8'h25,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};	// 16 char, 128 bits; line 1 of amplitude message

parameter	remoteL1		= {8'h20,8'h20,8'h52,8'h65,8'h6D,8'h6F,8'h74,8'h65,8'h20,8'h4D,8'h6F,8'h64,8'h65,8'h20,8'h20,8'h20};	// 16 char, 128 bits; line 1 of remote notification

parameter	errorL1		= {8'h45,8'h72,8'h72,8'h6F,8'h72,8'h20,8'h20,8'h20,8'h20,8'h45,8'h72,8'h72,8'h6F,8'h72,8'h20,8'h20};	// 16 char, 128 bits; line 1 of error notification

// DDS parameters
parameter	cfr1Man	= 32'b00000010000000000000001001000010;	// manual OSK enabled (amplitude control)
parameter	cfr2Man	= 24'b000000000000000000000100;			// clk mult = OFF; VCO higher range set
parameter	fastscw = 40'h017FFFFFFF;						// fast sweep control word


always @(posedge clk)
	begin
		counter			<= counter + 1;
		
		
		////////////////////////////////////////////
		// Set remote, lock, and display triggers //
		////////////////////////////////////////////
		if (counter[17])	// limits how frequently changes occur (prevent manual switch "bouncing" effects)
			begin
				remoteEnable	<= (~ (FPGA_switch0));	// switch determines remote or manual mode
				dispSelTrig		<= (~ (FPGA_button2));		// button triggers display menu change
				downTrig			<= (~ (FPGA_button0));		// button triggers down increment
				upTrig			<= (~ (FPGA_button1));		// button triggers up increment
			end
	
	
		////////////////////////////
		// Set LCD Display Choice //
		////////////////////////////
		// In manual control mode, set the status to display.
		case (dispSetStep)
			2'h0:	// check if in remote mode
				begin
					if (remoteEnable)	lcdDisplay	<= 4'hF;
					else	dispSetStep	<= 2'h1;
				end
			2'h1:	// look for trigger
				begin
					if (dispSelTrig)
						begin
							if(lcdDisplay < 4'h2) lcdDisplay	<= lcdDisplay + 4'h1; //changed the first 4'h2 to 4'h1
							else	lcdDisplay	<= 4'h0;
							dispSetWaitCnt	<= 26'h0000000;
							dispSetStep		<= 2'h2;
						end
					else	
						begin
							if (lcdDisplay == 4'hF)	lcdDisplay	<= 4'h0;	// if just ended remote mode, then reset lcdDisplay
							dispSetStep	<= 2'h0;
						end
				end
			2'h2:	// wait before doing another change
				begin
					if (dispSetWaitCnt[25] == 1)	dispSetStep	<= 2'h0;		// if we've waited long enough, reinitialize
					else	dispSetWaitCnt	<= dispSetWaitCnt + 26'h0000001;	// otherwise, wait longer
				end
			default:	dispSetStep	<= 2'h0;
		endcase


		///////////////////////
		// LCD Write Control //
		///////////////////////
		case (lcdAction)
			0:	// Check initialization of LCD.
				begin	
						case (switch)
							0:	lcdAction	<= 3;	// If already initialized, and switch low, then move on.
							1:	begin
									lcdWelcome	<= 0;	// And reset welcome message.
									case (resetStep)
										0:
											begin
												lcdData			<= 8'hFE;	// command prefix
												lcdSend[2:1]	<= 2'b00;	// send instruction to LCD
												lcdAction		<= 1;		// go to write action
												resetStep		<= 1;
											end
										1:
											begin
												lcdData			<= 8'h51;	// clear command
												lcdSend[2:1]	<= 2'b00;
												lcdAction		<= 1;
												resetStep		<= 0;
											end
										default
											begin
												resetStep	<= 0;
											end
									endcase	
								end
						endcase
				end
			1:	// Trigger a write to the LCD.
				begin
					if (lcdcsb	== 0)	lcdSend[0]	<= 0;	// If currently writing something to the LCD, wait here (turn off write).
					else
						begin
							lcdSend[0]	<= 1;
							lcdAction	<= 2;
						end
				end
			2:	// Clean-up after write, and wait before moving on (leave extra time between writes to LCD).
				begin
					if (waitCnt[17])	// for 50 MHz clk, 100 000 cycles is 2 ms.
						begin
							waitCnt		<= 0;
							lcdAction	<= 0;
						end
					else	waitCnt	<= waitCnt	+ 1;
					lcdSend[0]	<= 0;
				end
			3:	// Display "welcome" message.
				begin
					if (lcdWelcome)	lcdAction	<= 4;	// If already displayed welcome message, move on.
					else
						case (lcdWStep)
							0:	// get welcome message string
								begin
									lcdMessL1	<= welcomeL1;	// 16 characters, 128 bits
									charCnt		<= 0;
									lcdWStep	<= 1;
								end
							1:	// send character
								begin
									lcdData			<= lcdMessL1[127:120];	// get next character
									lcdMessL1		<= lcdMessL1 << 8;		// shift register by 8
									lcdSend[2:1]	<= 2'b01;				// send data to LCD
									lcdAction		<= 1;					// go to write section
									if	(charCnt == 4'hF)					// if this is the last (16th) character, move on
										begin
											charCnt		<= 0;
											lcdWStep	<= 2;
										end
									else	charCnt	<= charCnt + 1;		// if not done reading, just increment
								end
							2:	// hold welcome message
								begin
									if (waitCnt[28] == 0)	waitCnt	<= waitCnt + 1;// display the welcome message for a while before moving on
									else
										begin
											waitCnt		<= 0;
											lcdWelcome	<= 1;		// signals welcome message complete
											lcdWStep	<= 0;		// reinitialize, in case reset
											lcdAction	<= 1;
										end
								end
							default:
								begin
									lcdWelcome	<= 0;
									lcdWStep	<= 0;
									lcdAction	<= 0;
								end
						endcase
				end
			4:	// Display status of DDS
				case (lcdStep)
					0:	// return to first line, first character address
						case (resetStep)
							0:
								begin
									lcdData			<= 8'hFE;	// command prefix
									lcdSend[2:1]	<= 2'b00;	// send instruction to LCD
									lcdAction		<= 1;		// go to write action
									resetStep		<= 1;
									end
							1:
								begin
									lcdData			<= 8'h46;	// home command
									lcdSend[2:1]	<= 2'b00;
									lcdAction		<= 1;
									resetStep		<= 0;
									lcdStep			<= 1;
								end
							default
								begin
									lcdStep		<= 0;
									resetStep	<= 0;
								end
						endcase
					1:	// choose which status to display
						case (lcdDisplay)
							4'h0:	// display frequency (MHz)
								case (lcdDispStep)
									1'b0:	// get general freq format
										begin
											lcdMessL1	<= freqL1;
											lcdDispStep	<= 1'b1;
										end
									1'b1:	// get current frequency in BCD
										begin
											lcdMessL1[75:72]	<= freqBCD[23:20];	// Note that since general format already has
											lcdMessL1[67:64]	<= freqBCD[19:16];	//	these as numbers, just need the last 4 bits
											lcdMessL1[59:56]	<= freqBCD[15:12];	//	of each to determine which number.
											lcdMessL1[43:40]	<= freqBCD[11:8];
											lcdMessL1[35:32]	<= freqBCD[7:4];
											lcdMessL1[27:24]	<= freqBCD[3:0];
											lcdDispStep			<= 1'b0;
											lcdStep				<= 2;
										end
									default:	lcdDispStep	<= 1'b0;
								endcase
							4'h1:	// display frequency (kHz)
								case (lcdDispStep)
									1'b0:	// adjust general freq format
										begin
											lcdMessL1[127:52]	<= freqL1[127:52];
											lcdMessL1[51:48]	<= 4'h0;
											lcdMessL1[47:24]	<= freqL1[47:24];
											lcdMessL1[23:16]	<= 8'h6B;
											lcdMessL1[15:0]		<= freqL1[15:0];
											lcdDispStep	<= 1'b1;
										end
									1'b1:	// get current frequency in BCD
										begin
											lcdMessL1[75:72]	<= freqBCD[23:20];	// Note that since general format already has
											lcdMessL1[67:64]	<= freqBCD[19:16];	//	these as numbers, just need the last 4 bits
											lcdMessL1[59:56]	<= freqBCD[15:12];	//	of each to determine which number.
											lcdMessL1[43:40]	<= freqBCD[11:8];
											lcdMessL1[35:32]	<= freqBCD[7:4];
											lcdMessL1[27:24]	<= freqBCD[3:0];
											lcdDispStep			<= 1'b0;
											lcdStep				<= 2;
										end
									default:	lcdDispStep	<= 0;
								endcase
							4'h2:	// display amplitude
								case (lcdDispStep)
									1'b0:	// get general amp format
										begin
											lcdMessL1	<= ampL1;	
											lcdDispStep	<= 1'b1;
										end
									1'b1:	// get current amplitude in BCD
										begin
											lcdMessL1[75:72]	<= ampBCD[11:8];
											lcdMessL1[67:64]	<= ampBCD[7:4];
											lcdMessL1[59:56]	<= ampBCD[3:0];
											lcdDispStep			<= 1'b0;
											lcdStep				<= 2;
										end
									default:	lcdDispStep	<= 0;
								endcase
							4'hF:	// remote-mode
								begin
									lcdMessL1	<= remoteL1;
									lcdDispStep	<= 1'b0;
									lcdStep		<= 2;
								end
							default:	// display error message
								begin
									lcdMessL1	<= errorL1;
									lcdDispStep	<= 1'b0;
									lcdStep		<= 2;
								end
						endcase
					2:	// send characters
						begin
							lcdData			<= lcdMessL1[127:120];	// get next character
							lcdMessL1		<= lcdMessL1 << 8;		// shift register by 8
							lcdSend[2:1]	<= 2'b01;				// send data to LCD
							lcdAction		<= 1;					// go to write section
							if	(charCnt == 4'hF)					// if this is the last (16th) character, repeat status display
								begin
									charCnt	<= 0;
									lcdStep	<= 0;
								end
							else	charCnt	<= charCnt + 1;		// if not done reading, just increment
						end
					default:
						begin
							lcdStep		<= 0;
							lcdAction	<= 0;
						end
				endcase
			default:
				begin
					waitCnt		<= 0;
					lcdData		<= 0;
					lcdSend		<= 0;
					lcdAction	<= 0;
				end
		endcase
	end


always @(posedge fclk)
	begin
		////////////////////////
		// DDS Initialization //
		////////////////////////
		// The initialization sequence first applies a RESET signal to the DDS, then writes a default CFR1 and CFR2
		// to the DDS (these defaults are used in manual mode).
		case (ddsInitStep)
			0:	// reset DDS
				begin
					ddsInit		<= 0;
					ddsrst		<= 1;
					cfr1Init	<= 0;
					cfr2Init	<= 0;
					ddsWaitCnt	<= 0;
					ddsInitStep	<= 1;
				end
			1:	// wait (> 1 ms; DDS wake-up time)
				begin
					if (ddsWaitCnt[23] == 1)
						begin
							ddsWaitCnt	<= 0;
							ddsInitStep	<= 2;
						end
					else	ddsWaitCnt	<= ddsWaitCnt + 1;
				end
			2:	// write default CFRs
				begin
					ddsrst		<= 0;
					cfr1Init	<= cfr1Man;
					cfr2Init	<= cfr2Man;
					ddsWaitCnt	<= 0;
					ddsInitStep	<= 3;
				end
			3:	// wait
				begin
					if (ddsWaitCnt[23] == 1)
						begin
							ddsWaitCnt	<= 0;
							ddsInitStep	<= 4;
						end
					else	ddsWaitCnt	<= ddsWaitCnt + 1;
				end
			4:	// initialization complete
				begin
					ddsInit		<= 1;
					ddsInitStep	<= 5;
				end
			5:	// look for manual reinitialization
				begin
					if (switch)
						begin
							ddsInit		<= 0;
							ddsInitStep	<= 0;
						end
				end
			default:	ddsInitStep	<= 0;
		endcase
		
	
		//////////////////
		// Data for DDS //
		//////////////////
		// --If in manual control, use registers based on user input.
		// --If in remote control, use registers updated by TTL advanced FIFOs maintained by USB communication.
		// --Compare previous cfr, frequency, phase, and amplitude registers to current ones.  If the current ones are different, flag the DDS write section.
		case (remoteEnable)
			1'b0:	// manual control
				begin
					if (ddsInit)	// check if DDS initialization completed
						begin
							if (~ddsWriteStatus)	// check DDS register updates complete
								begin
									cfr1	<= cfr1Man;
									cfr2	<= cfr2Man;
									asf	<= ampMan;
									ftw0	<= freqMan;
									ps0	<= 1'b0;
									firstSwitch <= 1'b1;
								end
						end
					else
						begin
							cfr1	<= cfr1Init;
							cfr2	<= cfr2Init;
							firstSwitch <= 1'b1;
						end
				
					if (cfr1 == prvcfr1)	ddsFlags[7]	<= 0;
					else	ddsFlags[7]	<= 1;
					
					if (cfr2 == prvcfr2)	ddsFlags[6]	<= 0;
					else	ddsFlags[6]	<= 1;
					
					if (asf == prvasf)		ddsFlags[5]	<= 0;
					else	ddsFlags[5]	<= 1;
					
					if (ftw0 == prvftw0)	ddsFlags[4]	<= 0;
					else	ddsFlags[4]	<= 1;
					
					ddsFlags[3:0]	<= 0;	// for registers not accessible manually, do not bother with looking for updates
				end
			1'b1:	// remote control
				begin
					if (ddsInit)	// check if DDS initialization completed
						begin
							cfr1	<= rcfr1;
							//cfr2	<= rcfr2;	// Really no reason to change CFR2 remotely, so we'll remove it.
							asf	<= nasf;
							ftw0	<= nftw0;
							pow	<= npow;
							ftw1	<= nftw1;
							nlscw	<= nnlscw;
							plscw	<= nplscw;
							dps0	<= nps0;		// delay PS0 assignment by 1 clk cycle
							ps0	<= dps0;
							if (firstSwitch)
								begin
									syncStep	<= 2'h1;
									firstSwitch <= 1'b0;
								end
						end
					else
						begin
							cfr1	<= cfr1Init;
							cfr2	<= cfr2Init;
						end
				
					if (cfr1 == prvcfr1)	ddsFlags[7]	<= 0;
					else	ddsFlags[7]	<= 1;
					
					if (cfr2 == prvcfr2)	ddsFlags[6]	<= 0;
					else	ddsFlags[6]	<= 1;
					
					if (asf == prvasf)		ddsFlags[5]	<= 0;
					else	ddsFlags[5]	<= 1;
					
					if (ftw0 == prvftw0)	ddsFlags[4]	<= 0;
					else	ddsFlags[4]	<= 1;
					
					if (pow == prvpow)		ddsFlags[3]	<= 0;
					else	ddsFlags[3]	<= 1;
					
					if (ftw1 == prvftw1)	ddsFlags[2]	<= 0;
					else	ddsFlags[2]	<= 1;
					
					if (nlscw == prvnlscw)	ddsFlags[1]	<= 0;
					else	ddsFlags[1]	<= 1;
					
					if (plscw == prvplscw)	ddsFlags[0]	<= 0;
					else	ddsFlags[0]	<= 1;
				end
		endcase
		
		
		///////////////
		// DDS write //
		///////////////
		// Use a case statement to detemine which register to write to the DDS.  To give precedence to particular registers, will use a case statement with
		// certain bits as "don't care" (this way we won't have to write out ALL possible situations to give precedence).
		
		if (ddscs)				// only start a write operation if the DDS module is ready (i.e. not currently writing something to the DDS)
			casez (ddsFlags)
				9'b?1???????:	// cfr1 needs updating (for now, don't care if other registers do or not)
					begin
						ddsSend		<= 5'h10;
						prvcfr1		<= cfr1;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?01??????:	// cfr2 update (only if cfr1 update not needed)
					begin
						ddsSend		<= 5'h11;
						prvcfr2		<= cfr2;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?00?1????:	// ftw0 update (only if cfr1 and cfr2 not needed)
					begin
						ddsSend		<= 5'h14;
						prvftw0		<= ftw0;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?00?01???:	// pow update (see above to get trend of precedence)
					begin
						ddsSend		<= 5'h15;
						prvpow		<= pow;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?00?001??:	// ftw1 update (see above to get trend of precedence)
					begin
						ddsSend		<= 5'h16;
						prvftw1		<= ftw1;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?00?0001?:	// nlscw update (see above to get trend of precedence)
					begin
						ddsSend		<= 5'h17;
						prvnlscw	<= nlscw;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?00?00001:	// plscw update (see above to get trend of precedence)
					begin
						ddsSend		<= 5'h18;
						prvplscw	<= plscw;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b?00100000:	// asf update (only if no other registers need updating; this is last because of potential continuous changes while locked)
					begin
						ddsSend		<= 5'h12;
						prvasf		<= asf;
						ddsioupdate	<= 1'b0;
						ddsFlags[8]	<= 1'b1;
					end
				9'b100000000:	// all registers have been updated; do io-update
					case (syncStep)
						2'h0:	// if in remote/table mode, wait until external or software trigger is received; if not, do it immediately;
							begin
								if ((remoteEnable == 1'b1) && (extTrig == 1'b0))	syncStep	<= 2'h0;
								//  && (softTrig == 1'b0)
								else	syncStep	<= 2'h1;
							end
						2'h1:	// io-update high;
							begin
								ddsioupdate	<= 1'b1;
								if (ddsInit)	syncPause	<= 4'h2;
								else			syncPause	<= 4'hF;	// If the internal clk mult not running yet, need to have IO-UPDATE pulse longer.
								if (syncWait == syncPause)
									begin
										syncStep	<= 2'h2;
										syncWait	<= 4'h0;
									end
								else	syncWait	<= syncWait	+ 4'h1;
							end
						2'h2:	// io-update low; call next FIFO line; briefly set sweep direction opposite (reset/ready sweep) if in linear sweep mode.
							begin
								ddsioupdate	<= 1'b0;
								//if (cfr1[21])	ddsps0	<= ~ps0;
								ddsFlags[8]	<= 1'b0;
								getNextLine	<= 1'b1;
								syncStep	<= 2'h0;
							end
						default:	
							begin
								syncStep	<= 2'h0;
								syncWait	<= 4'h0;
							end
					endcase
				9'b000000000:	// all registers are up-to-date, so we can safely indicate the proper sweep direction
					begin
						ddsioupdate	<= 1'b0;
						ddsps0		<= ps0;
						if (extTrig)	getNextLine	<= 1'b1;	// makes sure we do not latch here, in case just switched to table mode, or previous line identical to this one
						else	getNextLine	<= 1'b0;
					end
				default:
					begin
						ddsioupdate	<= 1'b0;
						ddsSend		<= 5'h00;	// do nothing
					end
			endcase
		else	ddsSend	<= 5'h00;	// if in the middle of a write, just make sure the send command is reset
		
		
		////////////////////
		// Table/Sequence //
		////////////////////
		// Step through the table or sequence.  Uses external TTL trigger.
		//
		// First, some DDS "static" assignments (not updated by external triggers)
		ddsps1	<= 1'b0;
		ddsosk	<= 1'b0;
		
		// Clock syncronization.
		prevtrigger	<= {prevtrigger[1:0], triggerA};		// concatenate 2 newest triggerAs with current one
		extTrig		<= (prevtrigger[2] ^ prevtrigger[1]);	// xor
		
		// Delayed software trigger.
		case (sTrigStep)
			1'b0:	// look for software trigger from usb
				begin
					if (softTrig)	sTrigStep	<= 1'b1;
					softTrig0	<= 1'b0;
				end
			1'b1:	// wait
				begin
					if (sTrigWait[15])
						begin
							sTrigWait	<= 16'h0000;
							sTrigStep	<= 1'b0;
							softTrig0	<= 1'b1;
						end
					else	sTrigWait	<= sTrigWait + 16'h0001;
				end
		endcase
		
		// table/sequence state machines
		case (rcontrol)
			2'h0:	// table mode unlocked; uses "getNextLine" to look-ahead and prepare the next line of the FIFO.
				begin
					case (tableStep)
						0:	// look for get next line flag
							begin
								if (getNextLine && (empty[2:0] == 3'b000))
									begin
										readReq[6:0]	<= 7'b1111111;
										tableStep		<= 1;
									end
								else	tableStep	<= 0;
							end
						1:	// end read request
							begin
								readReq[6:0]	<= 7'b0000000;
								readReq[6]		<= 1'b0;
								tableStep		<= 2;
							end
						2:	// assign new FIFO data
							begin
								nasf		<= qasf;
								nftw0		<= qftw0;
								npow		<= qpow;
								nftw1		<= qftw1;
								// Set sweep control words based on direction of next sweep (to make sure FTW1 is reached before doing a downward sweep).
								if (qps0)
									begin
										nnlscw	<= fastscw;	// fast
										nplscw	<= qlscw;
									end
								else
									begin
										nnlscw	<= qlscw;
										nplscw	<= fastscw;	// fast
									end
								ntime		<= qtime;
								ips0		<= qps0;		// set "intermediate" value for ps0;
								tableStep	<= 0;
							end
					endcase
					
					// Set the "new" value (using the intermediate value above) for PS0 only after an external trigger is received.
					//	We do it this way, because unlike the other DDS parameters, the PS0 value is assigned immediately (no hold for io-update).
					//	Also, we can then check what the FIRST line is, and set the direction opposite (to allow for downward sweeps on the first line).
					if (extTrig)
						begin
							if ((depth5[7:0] == rlines[7:0]) && (rlines[9:8] == 2'b00))	nps0	<= ~qps0;	// If there has not yet been a read request (i.e. initialization trigger), then we know the FIFO is showing the first line, even though we haven't actually read it yet, so take the opposite value
							else	nps0	<= ips0;
						end
				end
			2'h1:	// sequence mode unlocked
				case (seqStep)
					0:	// look for external trigger
						begin
							if (extTrig && (empty[2:0] == 3'b000))
								begin
									readReq[6:0]	<= 7'h7F;
									seqStep			<= 2;
								end
							else	seqStep	<= 0;
							rfTimer	<= 0;
						end
					1:	// read request (for multiple lines in sequence)
						begin
							if (empty[2:0] == 3'b000)
								begin
									readReq[6:0]	<= 7'h7F;
									seqStep			<= 2;
								end
							else	seqStep	<= 0;	// if FIFO empty, abort
							rfTimer	<= 0;
						end
					2:	// end read request
						begin
							readReq[6:0]	<= 7'h00;
							rfTimer			<= rfTimer + 1;
							seqStep			<= 3;
						end
					3:	// assign new FIFO data for DDS
						begin
							nasf	<= qasf;
							nftw0	<= qftw0;
							npow	<= qpow;
							nftw1	<= qftw1;
							// Set sweep control words based on direction of next sweep (to make sure FTW1 is reached before doing a downward sweep).
							if (qps0)
								begin
									nnlscw	<= fastscw;	// fast
									nplscw	<= qlscw;
								end
							else
								begin
									nnlscw	<= qlscw;
									nplscw	<= fastscw;	// fast
								end
							nps0	<= qps0;
							ntime	<= qtime;
							rfTimer	<= rfTimer + 1;
							seqStep	<= 4;
						end
					4:	// wait for new FIFO data to be recognized by write flags
						begin
							rfTimer	<= rfTimer + 1;
							seqStep	<= 5;
						end
					5:	// wait for new FIFO data to be recognized by write flags
						begin
							rfTimer	<= rfTimer + 1;
							seqStep	<= 6;
						end
					6:	// wait for writing to DDS to cease
						begin
							if (ddsFlags == 9'b000000000)	seqStep	<= 7;
							rfTimer	<= rfTimer + 1;
						end
					7:	// wait specified amount of time before going to next step
						begin
							if (rfTimer < ntime)	rfTimer	<= rfTimer + 1;
							else	seqStep	<= 1;
						end
					default:	seqStep	<= 0;
				endcase
			2'h2:	// table mode locked
				case (lkTableStep)
					0:	// look for external trigger
						begin
							if (extTrig && (empty[2:0] == 3'b000))
								begin
									readReq[6:0]	<= 7'h7F;
									lkTableStep		<= 1;
								end
							else	lkTableStep	<= 0;
						end
					1:	// end read request
						begin
							readReq[6:0]	<= 7'h00;
							lkTableStep		<= 2;
						end
					2:	// assign new FIFO data
						begin
							nftw0		<= qftw0;
							npow		<= qpow;
							nftw1		<= qftw1;
							// Set sweep control words based on direction of next sweep (to make sure FTW1 is reached before doing a downward sweep).
							if (qps0)
								begin
									nnlscw	<= fastscw;	// fast
									nplscw	<= qlscw;
								end
							else
								begin
									nnlscw	<= qlscw;
									nplscw	<= fastscw;	// fast
								end
							ntime		<= qtime;
							nps0		<= qps0;
							lkTableStep	<= 0;
						end
					default:	lkTableStep	<= 0;
				endcase
			2'h3:	// sequence mode locked
				case (lkSeqStep)
					0:	// look for external trigger
						begin
							if (extTrig && (empty[2:0] == 3'b000))
								begin
									readReq[6:0]	<= 7'h7F;
									lkSeqStep		<= 2;
								end
							else	lkSeqStep	<= 0;
							rfTimer	<= 0;
						end
					1:	// read request (for multiple lines in sequence)
						begin
							if (empty[2:0] == 3'b000)
								begin
									readReq[6:0]	<= 7'h7F;
									lkSeqStep		<= 2;
								end
							else	lkSeqStep	<= 0;	// if FIFO empty, abort
							rfTimer	<= 0;
						end
					2:	// end read request
						begin
							readReq[6:0]	<= 7'h00;
							rfTimer			<= rfTimer + 1;
							lkSeqStep		<= 3;
						end
					3:	// assign new FIFO data
						begin
							nftw0		<= qftw0;
							npow		<= qpow;
							nftw1		<= qftw1;
							// Set sweep control words based on direction of next sweep (to make sure FTW1 is reached before doing a downward sweep).
							if (qps0)
								begin
									nnlscw	<= fastscw;	// fast
									nplscw	<= qlscw;
								end
							else
								begin
									nnlscw	<= qlscw;
									nplscw	<= fastscw;	// fast
								end
							nps0		<= qps0;
							ntime		<= qtime;
							rfTimer		<= rfTimer + 1;
							lkSeqStep	<= 4;
						end
					4:	// wait for new FIFO data to be recognized by write flags
						begin
							rfTimer		<= rfTimer + 1;
							lkSeqStep	<= 5;
						end
					5:	// wait for new FIFO data to be recognized by write flags
						begin
							rfTimer		<= rfTimer + 1;
							lkSeqStep	<= 6;
						end
					6:	// wait for writing to DDS to cease
						begin
							if (ddsFlags == 9'b000000000)	lkSeqStep	<= 7;
							rfTimer	<= rfTimer + 1;
						end
					7:	// wait specified amount of time before going to next step
						begin
							if (rfTimer < ntime)	rfTimer	<= rfTimer + 1;
							else	lkSeqStep	<= 1;
						end
					default:	lkSeqStep	<= 0;
				endcase
		endcase
		
		
		///////////
		// Misc. //
		///////////
		ddsWriteStatus	<= (ddsFlags[8] | ddsFlags[7] | ddsFlags[6] | ddsFlags[5] | ddsFlags[4] | ddsFlags[3] | ddsFlags[2] | ddsFlags[1] | ddsFlags[0]);
	end


//////////////////
// Ext. Trigger //
//////////////////
// Update a register based on an input TTL pulse.  Triggers FPGA to go to the next line in the table, or run the table as a sequence.
// Note that doing it based on the rising edge of in2 rather than based on the state (high or low) is designed 
// to prevent double-triggering a sequence as a result of the time of the in2 pulse being longer than the DDS 
// sequence time.
//	Also note that this signal is synchronized with clk in the above "always" statement.
always @(posedge in2 | ~ TTLbutton)
	begin
		triggerA	<= ~triggerA;
	end

//////////////////////////////
// Multiply Clock using PLL //
//////////////////////////////
// Use the PLL Megafunction to get multiples of the 50 MHz input clock.
triplepll	getTripleClks(.inclk0(clk),
						  .c0(sclk),
						  .c1(fclk),
						  .c2(ssclk),
						  .locked(pllLocked));


////////////////////////
// DDS Writing Module //
////////////////////////
dds_write	writeDDS(.clk(fclk),
					 .send(ddsSend),
					 .cs(ddscs),
					 .sclk(ddssclk),
					 .sdio(ddssdio),
					 .cfr1(cfr1),
					 .cfr2(cfr2),
					 .asf(asf),
					 .ftw0(ftw0),
					 .pow(pow),
					 .ftw1(ftw1),
					 .nlscw(nlscw),
					 .plscw(plscw));


////////////////////////
// LCD Writing Module //
////////////////////////
// Given data and a trigger, controls the writing action to the LCD display.
lcd_write	writeLCD(.clk(clk),
					 .recData(lcdData),
					 .send(lcdSend),
					 .csb(lcdcsb),
					 .scl(lcdscl),
					 .si(lcdsi));


/////////////////////////////////////
// Manual Frequency Control Module //
/////////////////////////////////////
// Changes frequency sent to DDS (and displayed) based on user input (button pushing).
manual_freq	manualFreq(.clk(clk),
					   .trigup(upTrig),
					   .trigdown(downTrig),
					   .active(lcdDisplay),
					   .freqDDS(freqMan),
					   .freqBCD(freqBCD));
					   

/////////////////////////////////////
// Manual Amplitude Control Module //
/////////////////////////////////////
// Changes amplitude sent to DDS (and displayed) based on user input (button pushing).
manual_amp	manualAmp(.clk(clk),
					  .trigup(upTrig),
					  .trigdown(downTrig),
					  .active(lcdDisplay),
					  .ampDDS(ampMan),
					  .ampBCD(ampBCD));
defparam	manualAmp.activeA	= 4'h2;


////////////////////////
// FIFO Megafunctions //
////////////////////////
// Call all of the FIFO megafunctions needed.
fifo_w14_d1024	fifo0(.clock(fclk),
					  .data(rasf),
					  .rdreq(readReq[0]),
					  .sclr(sclr[0]),
					  .wrreq(writeReq[0]),
					  .empty(empty[0]),
					  .full(full[0]),
					  .q(qasf),
					  .usedw(depth0));

fifo_w32_d1024	fifo1(.clock(fclk),
					  .data(rftw0),
					  .rdreq(readReq[1]),
					  .sclr(sclr[1]),
					  .wrreq(writeReq[1]),
					  .empty(empty[1]),
					  .full(full[1]),
					  .q(qftw0),
					  .usedw(depth1));

fifo_w14_d1024	fifo2(.clock(fclk),
					  .data(rpow),
					  .rdreq(readReq[2]),
					  .sclr(sclr[2]),
					  .wrreq(writeReq[2]),
					  .empty(empty[2]),
					  .full(full[2]),
					  .q(qpow),
					  .usedw(depth2));

fifo_w32_d256	fifo3(.clock(fclk),
					  .data(rftw1),
					  .rdreq(readReq[3]),
					  .sclr(sclr[3]),
					  .wrreq(writeReq[3]),
					  .empty(empty[3]),
					  .full(full[3]),
					  .q(qftw1),
					  .usedw(depth3));

fifo_w40_d256	fifo4(.clock(fclk),
					  .data(rlscw),
					  .rdreq(readReq[4]),
					  .sclr(sclr[4]),
					  .wrreq(writeReq[4]),
					  .empty(empty[4]),
					  .full(full[4]),
					  .q(qlscw),
					  .usedw(depth4));

fifo_w1_d256	fifo5(.clock(fclk),
					  .data(rps0),
					  .rdreq(readReq[5]),
					  .sclr(sclr[5]),
					  .wrreq(writeReq[5]),
					  .empty(empty[5]),
					  .full(full[5]),
					  .q(qps0),
					  .usedw(depth5));

fifo_w32_d512	fifo6(.clock(fclk),
					  .data(rtime),
					  .rdreq(readReq[6]),
					  .sclr(sclr[6]),
					  .wrreq(writeReq[6]),
					  .empty(empty[6]),
					  .full(full[6]),
					  .q(qtime),
					  .usedw(depth6));


///////////////////////
// USB Communication //
///////////////////////
// The USB communication module gets data from the PC, and updates the FIFO registers.
usb_registers	usbComm(.clk(fclk),
						.RxD(usbtxd),
						.rcontrol(rcontrol),
						.rcfr1(rcfr1),
						.rcfr2(rcfr2),
						.rasf(rasf),
						.rftw0(rftw0),
						.rpow(rpow),
						.rftw1(rftw1),
						.rlscw(rlscw),
						.rps0(rps0),
						.rtime(rtime),
						.rlines(rlines),
						.depth0(depth0),
						.depth1(depth1),
						.depth2(depth2),
						.depth3(depth3),
						.depth4(depth4),
						.depth5(depth5),
						.depth6(depth6),
						.writeReq(writeReq),
						.sclr(sclr),
						.softTrig(softTrig));


						
/////////////////////
// Pin Assignments //
/////////////////////
// LCD Pins
assign	ext[0]	= lcdcsb;
assign	ext[1]	= lcdscl;
assign	ext[2]	= lcdsi;

// DDS Pins
assign	out3[7] = ddsrst;
assign	out3[6] = ddscs;
assign	out3[5] = ddssclk;
assign	out3[4]	= ddssdio;
assign	out3[3] = ddsosk;
assign	out3[2]	= ddsps0;
assign	out3[1] = ddsps1;
assign	out3[0]	= ddsioupdate;

// Misc Pins
assign	LED			= remoteEnable;
assign   usbtalk 		= empty[0];
assign   TTLLED		= triggerA;
assign	out1[1]		= extTrig;
assign	out1[0]		= ddsioupdate;

endmodule
