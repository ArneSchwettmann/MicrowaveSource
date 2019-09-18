// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group

// Module Name:
// usb_registers

// Project Name:
// dds_driver

// Target Devices:
// DE0-CV Cyclone V FPGA board


// Notes
// This module should receive a string from the PC via USB, parse the data, and store it in the appropriate register.
// Expect to see data of the form:
//		LNXXXXXX...XX<CR/LF>
// Here, L announces that the following data is to be used to update one of the registers, N (0-?) determines which of 
// the 16 possible registers to update, and XXXXXX...XX is the data to store in the register.  We only use the last 4 binary bits 
// of each ascii character transmitted (and hence the valid choices for the characters below).  

// Note:  the reason we use only the last four bits is that to utilize all 8 bits of the character requires 
// some "non-printable" characters.

// If updating the frequency tuning word (ftw), 32 bits are needed (so 8 characters): in ascii each character should
// be 0-?.  If updating a phase offset word (pow), only 14 bits are needed, so only 4 characters are sent: in
// ascii these will be 0-3,0-?,0-?,0-?, since only 2 least significant bits of first X are needed.  Like the FTW, 
// the time data is now 32 bits, and so requires 8 characters.

// If updating the FIFO sequences, though, we need up to 10240 bits, meaning up to 2560 characters...
// ...thus the undefined number of X's...

module	usb_registers(clk,
					  RxD,
					  rcontrol,
					  rcfr1,
					  rcfr2,
					  rasf,
					  rftw0,
					  rpow,
					  rftw1,
					  rlscw,
					  rps0,
					  rtime,
					  rlines,
					  depth0,
					  depth1,
					  depth2,
					  depth3,
					  depth4,
					  depth5,
					  depth6,
					  writeReq,
					  sclr,
					  softTrig);

input			clk;
input			RxD;
input	[9:0]	depth0;
input	[9:0]	depth1;
input	[9:0]	depth2;
input	[7:0]	depth3;
input	[7:0]	depth4;
input	[7:0]	depth5;
input	[8:0]	depth6;

output	[1:0]		rcontrol;	// control word/bit that determines action upon receiving an external TTL trigger (e.g. step through whole table, or just next line)
output	[31:0]	rcfr1;		// remote CFR1 (single register per cycle)
output	[23:0]	rcfr2;		// remote CFR2 (single register per cycle)
output	[13:0]	rasf;			// ASF for FIFO; serves as PI controller set-point when in amp locked mode
output	[31:0]	rftw0;		// FTW0 for FIFO
output	[13:0]	rpow;			// POW for FIFO
output	[31:0]	rftw1;		// FTW1 for FIFO
output	[39:0]	rlscw;		// LSCW for FIFO (used for both PLSCW and NLSCW)
output				rps0;			// PS0 for FIFO
output	[31:0]	rtime;		// TIME for FIFO
output	[9:0]		rlines;		// number of items to write to the FIFO (lines in the table); max is 1024 (note: sweep-relevent FIFOs only 512 deep).
output	[6:0]		writeReq;	// To input port of FIFO megafunction; bits controlled by submodule.
output	[6:0]		sclr;			// To input port of FIFO megafunction; bits controlled by submodule.
output				softTrig;	// Software trigger for DDS (remote-table-mode, no amp lock).

// Output registers
reg		[1:0]		rcontrol;	// reg for output above
reg		[31:0]	rcfr1;		// reg for output above
reg		[23:0]	rcfr2;		// reg for output above
reg		[13:0]	rasf;			// reg for output above
reg		[31:0]	rftw0;		// reg for output above
reg		[13:0]	rpow;			// reg for output above
reg		[31:0]	rftw1;		// reg for output above
reg		[39:0]	rlscw;		// reg for output above
reg					rps0;			// reg for output above
reg		[31:0]	rtime;		// reg for output above
reg		[9:0]		rlines;		// reg for output above
reg		[8:0]		rlines2;		// number of items (lines) to write to the mid-size (512) FIFOs
reg		[7:0]		rlines3;		// number of items (lines) to write to the small-size (256) FIFOs
reg					softTrig;	// reg for output above

// FIFO registers/wires
reg		[6:0]	clrFifo;	// High bit is a flag to clear that FIFO.
reg		[6:0]	trigWrite;	// High bit is a flag to begin a write to that FIFO.

// Wires for async_receiver
wire			usbready;	// Signal from async_receiver, announcing when the usb data has been received.
wire	[7:0]	usbdata;	// Connects to register where the data obtained from the usb by async_receiver is stored.

// State machine registers
reg				newreg;		// Indicates if we are just beginning data transfer for a new register (used to tell whether or not the FIFO should be cleared).
reg		[3:0]	regnum;		// Indicates which register is being updated.
reg		[3:0]	usbstate;	// Status of the usb data transfer state machine.

// Define a couple of parameters specific to the operation of this board/module.
parameter	ClkFreq		= 100000000;		// Make sure this matches the clock frequency on your board.
parameter	BaudRate	= 2000000;			// Define the rate you want to transfer information to/from the PC.

// Receive the data from the USB port.
async_receiver 	getusbdata(.clk(clk), .RxD(RxD), .RxD_data_ready(usbready), .RxD_data(usbdata));
defparam		getusbdata.ClkFrequency 	= ClkFreq;
defparam		getusbdata.Baud 			= BaudRate;


//////////////////////////////////////////
// State machine for USB communication. //
//////////////////////////////////////////
// Expects data of the form LNXXXXYYYYZZ...ZZ<CR/LF>, where the XXXXYYYYZZ...ZZ data is stored into a register/FIFO for later use.
always @(posedge clk)
	begin
		if (usbready)
			case (usbstate)
				4'h0:	// looking for an "L"
					begin
						if (usbdata == 8'h4C)	usbstate <= 4'h1;	// Looks for a capital L (in ascii... hex equivalent = 0x4C) to announce the beginning of the phase-updating string... next usbready should then change usbdata to N.
						else					usbstate <= 4'h0;	// If something else was sent, ignore it.
						
						// set max for mid-size FIFOs
						if (rlines[9] == 1'b0)		rlines2	<= rlines[8:0];
						else	rlines2	<= 9'h1FF;
						
						// set max for small-size FIFOs
						if (rlines[9:8] == 2'b00)	rlines3	<= rlines[7:0];
						else	rlines3	<= 8'hFF;
					end
				4'h1:	// should be the "N" to determine which register to update.
					begin
						regnum 		<= usbdata[3:0];
						usbstate	<= 4'h2;
						newreg		<= 1'b1;	// note that new register data starting
					end
				4'h2:	// "X" data
					begin
						case (regnum)
							4'h0:	begin
										rcontrol		<= usbdata[1:0];
										usbstate		<= 4'h0;							// done.
									end
							4'h1:	begin
										rcfr1[31:28]	<= usbdata[3:0];
										usbstate		<= 4'h3;
									end
							4'h2:	begin
										rcfr2[23:20]	<= usbdata[3:0];
										usbstate		<= 4'h3;
									end
							4'h3:	begin
										rasf[13:12]		<= usbdata[1:0];
										clrFifo[0]		<= newreg;							// if new register transfer just starting, clear the FIFO
										usbstate		<= 4'h3;
									end
							4'h4:	begin
										rftw0[31:28]	<= usbdata[3:0];
										clrFifo[1]		<= newreg;
										usbstate		<= 4'h3;
									end
							4'h5:	begin
										rpow[13:12]		<= usbdata[1:0];
										clrFifo[2]		<= newreg;
										usbstate		<= 4'h3;
									end
							4'h6:	begin
										rftw1[31:28]	<= usbdata[3:0];
										clrFifo[3]		<= newreg;
										usbstate		<= 4'h3;
									end
							4'h7:	begin
										rlscw[39:36]	<= usbdata[3:0];
										clrFifo[4]		<= newreg;
										usbstate		<= 4'h3;
									end
							4'h9:	begin
										rps0			<= usbdata[0];
										clrFifo[5]		<= newreg;
										trigWrite[5]	<= ~trigWrite[5];
										if	(depth5 + 1 == rlines3)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;						// otherwise, continue to get more data for the FIFO
									end
							4'hA:	begin
										rtime[31:28]	<= usbdata[3:0];
										clrFifo[6]		<= newreg;
										usbstate		<= 4'h3;
									end
							4'hB:	begin
										rlines[9:8]		<= usbdata[1:0];
										usbstate		<= 4'h3;
									end
							4'hF:	begin
										softTrig		<= usbdata[0];
										usbstate		<= 4'h0;
									end
							default:	usbstate	<= 4'h0;
						endcase
						newreg	<= 1'b0;													// end new register data flag
					end
				4'h3:	// "X" data
					begin
						case (regnum)
							4'h1:	rcfr1[27:24]	<= usbdata[3:0];
							4'h2:	rcfr2[19:16]	<= usbdata[3:0];
							4'h3:	rasf[11:8]		<= usbdata[3:0];
							4'h4:	rftw0[27:24]	<= usbdata[3:0];
							4'h5:	rpow[11:8]		<= usbdata[3:0];
							4'h6:	rftw1[27:24]	<= usbdata[3:0];
							4'h7:	rlscw[35:32]	<= usbdata[3:0];
							4'hA:	rtime[27:24]	<= usbdata[3:0];
							4'hB:	rlines[7:4]		<= usbdata[3:0];
						endcase
						usbstate	<= 4'h4;
					end
				4'h4:	// "X" data
					begin
						case (regnum)
							4'h1:	begin
										rcfr1[23:20]	<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'h2:	begin
										rcfr2[15:12]	<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'h3:	begin
										rasf[7:4]		<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'h4:	begin
										rftw0[23:20]	<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'h5:	begin
										rpow[7:4]		<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'h6:	begin
										rftw1[23:20]	<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'h7:	begin
										rlscw[31:28]	<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'hA:	begin
										rtime[23:20]	<= usbdata[3:0];
										usbstate		<= 4'h5;
									end
							4'hB:	begin
										rlines[3:0]		<= usbdata[3:0];
										usbstate		<= 4'h0;			// if rlines, then done.
									end
							default:	usbstate	<= 4'h0;
						endcase
					end
				4'h5:	// "X" data
					begin
						case (regnum)
							4'h1:	begin
										rcfr1[19:16]	<= usbdata[3:0];
										usbstate		<= 4'h6;
									end
							4'h2:	begin
										rcfr2[11:8]		<= usbdata[3:0];
										usbstate		<= 4'h6;
									end
							4'h3:	begin
										rasf[3:0]		<= usbdata[3:0];
										trigWrite[0]	<= ~trigWrite[0];
										if	(depth0 + 1 == rlines)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;								// otherwise, continue to get more data for the FIFO
									end
							4'h4:	begin
										rftw0[19:16]	<= usbdata[3:0];
										usbstate		<= 4'h6;
									end
							4'h5:	begin
										rpow[3:0]		<= usbdata[3:0];
										trigWrite[2]	<= ~trigWrite[2];
										if	(depth2 + 1 == rlines)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;								// otherwise, continue to get more data for the FIFO
									end
							4'h6:	begin
										rftw1[19:16]	<= usbdata[3:0];
										usbstate		<= 4'h6;
									end
							4'h7:	begin
										rlscw[27:24]	<= usbdata[3:0];
										usbstate		<= 4'h6;
									end
							4'hA:	begin
										rtime[19:16]	<= usbdata[3:0];
										usbstate		<= 4'h6;
									end
							default:	usbstate	<= 4'h0;
						endcase
					end
				4'h6:	// "X" data
					begin
						case (regnum)
							4'h1:	rcfr1[15:12]	<= usbdata[3:0];
							4'h2:	rcfr2[7:4]		<= usbdata[3:0];
							4'h4:	rftw0[15:12]	<= usbdata[3:0];
							4'h6:	rftw1[15:12]	<= usbdata[3:0];
							4'h7:	rlscw[23:20]	<= usbdata[3:0];
							4'hA:	rtime[15:12]	<= usbdata[3:0];
						endcase
						usbstate	<= 4'h7;
					end
				4'h7:	// "X" data
					begin
						case (regnum)
							4'h1:	rcfr1[11:8]		<= usbdata[3:0];
							4'h2:	rcfr2[3:0]		<= usbdata[3:0];
							4'h4:	rftw0[11:8]		<= usbdata[3:0];
							4'h6:	rftw1[11:8]		<= usbdata[3:0];
							4'h7:	rlscw[19:16]	<= usbdata[3:0];
							4'hA:	rtime[11:8]		<= usbdata[3:0];
						endcase
						if	(regnum == 4'h2)	usbstate	<= 4'h0;	// if rcfr2, then done
						else	usbstate	<= 4'h8;
					end
				4'h8:	// "X" data
					begin
						case (regnum)
							4'h1:	rcfr1[7:4]		<= usbdata[3:0];
							4'h4:	rftw0[7:4]		<= usbdata[3:0];
							4'h6:	rftw1[7:4]		<= usbdata[3:0];
							4'h7:	rlscw[15:12]	<= usbdata[3:0];
							4'hA:	rtime[7:4]		<= usbdata[3:0];
						endcase
						usbstate	<= 4'h9;
					end
				4'h9:	// "X" data
					begin
						case (regnum)
							4'h1:	begin
										rcfr1[3:0]	<= usbdata[3:0];
										usbstate	<= 4'h0;
									end
							4'h4:	begin
										rftw0[3:0]		<= usbdata[3:0];
										trigWrite[1]	<= ~trigWrite[1];
										if	(depth1 + 1 == rlines)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;								// otherwise, continue to get more data for the FIFO
									end
							4'h6:	begin
										rftw1[3:0]		<= usbdata[3:0];
										trigWrite[3]	<= ~trigWrite[3];
										if	(depth3 + 1 == rlines3)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;								// otherwise, continue to get more data for the FIFO
									end
							4'h7:	begin
										rlscw[11:8]		<= usbdata[3:0];
										usbstate		<= 4'hA;
									end
							4'hA:	begin
										rtime[3:0]		<= usbdata[3:0];
										trigWrite[6]	<= ~trigWrite[6];
										if	(depth6 + 1 == rlines2)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;								// otherwise, continue to get more data for the FIFO
									end
							default:	usbstate	<= 4'h0;
						endcase
					end
				4'hA:	// "X" data
					begin
						case (regnum)
							4'h7:	rlscw[7:4]	<= usbdata[3:0];
						endcase
						usbstate	<= 4'hB;
					end
				4'hB:	// "X" data
					begin
						case (regnum)
							4'h7:	begin
										rlscw[3:0]		<= usbdata[3:0];
										trigWrite[4]	<= ~trigWrite[4];
										if	(depth4 + 1 == rlines3)	usbstate	<= 4'h0;	// if after adding this line the depth of the FIFO will be at the max (rlines), then done
										else	usbstate	<= 4'h2;								// otherwise, continue to get more data for the FIFO
									end
							default:	usbstate	<= 4'h0;
						endcase
					end
				default:	usbstate	<= 4'h0;
			endcase
		else	softTrig	<= 1'b0;	// reset software trigger, to ensure only high for one clk cycle.
	end


//////////////////////////////////
// FIFO write-action submodules //
//////////////////////////////////
fifo_write	writeFIFO0(.clk(clk),
					   .trigWrite(trigWrite[0]),
					   .clrFifo(clrFifo[0]),
					   .sclr(sclr[0]),
					   .writeReq(writeReq[0]));

fifo_write	writeFIFO1(.clk(clk),
					   .trigWrite(trigWrite[1]),
					   .clrFifo(clrFifo[1]),
					   .sclr(sclr[1]),
					   .writeReq(writeReq[1]));

fifo_write	writeFIFO2(.clk(clk),
					   .trigWrite(trigWrite[2]),
					   .clrFifo(clrFifo[2]),
					   .sclr(sclr[2]),
					   .writeReq(writeReq[2]));

fifo_write	writeFIFO3(.clk(clk),
					   .trigWrite(trigWrite[3]),
					   .clrFifo(clrFifo[3]),
					   .sclr(sclr[3]),
					   .writeReq(writeReq[3]));

fifo_write	writeFIFO4(.clk(clk),
					   .trigWrite(trigWrite[4]),
					   .clrFifo(clrFifo[4]),
					   .sclr(sclr[4]),
					   .writeReq(writeReq[4]));

fifo_write	writeFIFO5(.clk(clk),
					   .trigWrite(trigWrite[5]),
					   .clrFifo(clrFifo[5]),
					   .sclr(sclr[5]),
					   .writeReq(writeReq[5]));

fifo_write	writeFIFO6(.clk(clk),
					   .trigWrite(trigWrite[6]),
					   .clrFifo(clrFifo[6]),
					   .sclr(sclr[6]),
					   .writeReq(writeReq[6]));

endmodule
