// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group

// Module Name:
// dds_write

// Project Name:
// dds_driver

// Target Devices:
// DE0-CV Cyclone V FPGA board


// Notes
// Module designed to write data to the AD9954 DDS board.  It should:
//		--look for a "send" flag
//		--register this "send" signal within one clock cycle, as well as the data to be sent
//		--send the data to the DDS, with a serial "clock" of 25 MHz or less

module	dds_write(clk,
				  send,
				  cs,
				  sclk,
				  sdio,
				  cfr1,
				  cfr2,
				  asf,
				  ftw0,
				  pow,
				  ftw1,
				  nlscw,
				  plscw);

input				clk;
input	[4:0]		send;		// triggers a data write, and indicates which data to write
input	[31:0]	cfr1;		// CFR1 (control function register) to be sent to the DDS
input	[23:0]	cfr2;		// CFR2 (control function register) to be sent to the DDS
input	[13:0]	asf;		// ASF (amp scale factor) to be sent to the DDS
input	[31:0]	ftw0;		// FTW0 (freq tuning word) to be sent to the DDS
input	[13:0]	pow;		// POW (phase offset word) to be sent to the DDS
input	[31:0]	ftw1;		// FTW1 (freq tuning word) to be sent to the DDS (used only in linear sweep mode)
input	[39:0]	nlscw;	// NLSCW (negative linear sweep control word) to be sent to the DDS (used only in linear sweep mode)
input	[39:0]	plscw;	// PLSCW (positive linear sweep control word) to be sent to the DDS (used only in linear sweep mode)

output			cs;		// DDS chip-select pin (when low, indicates write is in progress
output			sclk;		// DDS serial clk pin
output			sdio;		// DDS sdio pin (data)

reg				cs;		// register for output above
reg				sclk;		// register for output above
reg				sdio;		// register for output above

reg	[3:0]		step;		// step we're at in the write procedure
reg	[47:0]	data;		// data to write to the DDS (including instruction byte)
reg	[5:0]		maxCnt;	// number of bits that need be transferred
reg	[5:0]		counter;	// number of bits that have been transferred

always @(posedge clk)
	case (step)
		0:	// look for write trigger
			case (send)
				5'h10:	// write CFR1
					begin
						data	<= {8'h00,cfr1,8'h00};	// most-significant byte is the instruction byte (register address; here = 8'h00)
						maxCnt	<= 40;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h11:	// write CFR2
					begin
						data	<= {8'h01,cfr2,16'h0000};
						maxCnt	<= 32;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h12:	// write ASF
					begin
						data	<= {8'h02,2'b00,asf,24'h000000};
						maxCnt	<= 24;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h14:	// write FTW0
					begin
						data	<= {8'h04,ftw0,8'h00};
						maxCnt	<= 40;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h15:	// write POW
					begin
						data	<= {8'h05,2'b00,pow,24'h000000};
						maxCnt	<= 24;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h16:	// write FTW1
					begin
						data	<= {8'h06,ftw1,8'h00};
						maxCnt	<= 40;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h17:	// write NLSCW
					begin
						data	<= {8'h07,nlscw};
						maxCnt	<= 48;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				5'h18:	// write PLSCW
					begin
						data	<= {8'h08,plscw};
						maxCnt	<= 48;
						counter	<= 0;
						cs		<= 0;
						step	<= 1;
					end
				default:	cs	<= 1;
			endcase
		1:	// check if write is complete; if not, get data and shift
			begin
				if	(counter == maxCnt)	// if counter equals maxCnt, then all bits have already been transferred, so reinitialize
					begin
						cs		<= 1;
						counter	<= 0;
						step	<= 0;
					end
				else
					begin
						sdio	<= data[47];
						data	<= data << 1;
						counter	<= counter + 1;
						step	<= 2;
					end
			end
		2:	// tick
			begin
				sclk	<= 0;
				step	<= 3;
			end
		3:	// tock
			begin
				sclk	<= 1;
				step	<= 4;
			end
		4:	// tick
			begin
				sclk	<= 0;
				step	<= 1;
			end
		default:	step	<= 0;
	endcase

endmodule
