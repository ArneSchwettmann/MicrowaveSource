// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group

// Module Name:
// manual_freq

// Project Name:
// dds_driver

// Target Devices:
// DE0-CV Cyclone V FPGA board

// Notes:
// This module is designed to update the frequency parameter for the DDS based on manual input by the user.  The user can 
// change the frequency either in 1 MHz or 1 kHz increments.
//	For a 360 MHz internal DDS clock, the appropriate conversion factors are:
//	1 MHz = 11930465 = 24'b101101100000101101100001		(only off by ~4 Hz after 180 increments)
//	1 kHz = 11930    = 14'b10111010011010				(only off by ~39 Hz after 1000 increments; off by ~7 kHz after 180*1000 increments)
//	For a 400 MHz internal DDS clock, the appropriate conversion factors are:
//	1 MHz = 10737418 = 32'h00A3D70A		(only off by 4 Hz after 200 increments)
//	1 kHz = 10737    = 32'h000029F1		(only off by ~39 Hz after 1000 increments; off by ~8 kHz after 200*1000 increments)

module	manual_freq(clk,
					trigup,
					trigdown,
					active,
					freqDDS,
					freqBCD);

input					clk;
input					trigup;		// triggers an increment frequency by one unit
input					trigdown;	// triggers a decrement frequency by one unit
input		[3:0]		active;		// sets status to update; 4'h0: frequency changed by 1 MHz on trigger; 4'h1: freq changed by 1 kHz on trigger

output	[31:0]	freqDDS;		// output frequency to be sent to the DDS (binary)
output	[23:0]	freqBCD;		// output frequency to be sent to the LCD (BCD)

reg		[4:0]		counter;		// counter for the shift procedure
reg		[31:0]	freqDDS;		// register for the output above
reg		[23:0]	freqBCD;		// register for the output above
reg		[17:0]	freqBIN;		// the binary representation of the freq in kHz (NOT the same binary number as the FTW sent to the DDS)
reg		[41:0]	freqINT;		// the intermediate register that will be used to do the binary to BCD conversion.
reg		[3:0]	freqChange;		// determines how to change the frequency
reg		[2:0]		step;			// step in changing the frequency
reg		[1:0]		convStep;	// step in converting from binary to BCD
reg		[23:0]	waitCnt;		// counter to wait some time between triggers

parameter	MHz		= 32'h00A3D70A;
parameter	kHz		= 32'h000029F1;

always @(posedge clk)
	begin
		freqChange	<= {(active == 4'h0), (active == 4'h1), trigup, trigdown};
		
		case (step)
			0:	// look for frequency change trigger
				case (freqChange)
					4'b0110:	// +1 kHz
						begin
							if (freqDDS <= (32'h7FFFFFFF - kHz))	// check that we are not at max freq
								begin
									freqDDS	<= freqDDS + kHz;
									freqBIN	<= freqBIN + 1;
								end
							step	<= 1;
						end
					4'b0101:	// -1 kHz
						begin
							if (freqDDS >= kHz)	// check that we are not at min freq
								begin
									freqDDS	<= freqDDS - kHz;
									freqBIN	<= freqBIN - 1;
								end
							step	<= 1;
						end
					4'b1010:	// +1 MHz
						begin
							if (freqDDS <= (32'h7FFFFFFF - MHz))	// check that we are not at max freq
								begin
									freqDDS	<= freqDDS + MHz;
									freqBIN	<= freqBIN + 1000;
								end
							step	<= 1;
						end
					4'b1001:	// -1 MHz
						begin
							if (freqDDS >= MHz)	// check that we are not at min freq
								begin
									freqDDS	<= freqDDS - MHz;
									freqBIN	<= freqBIN - 1000;	// will re-convert to BCD at later step
								end
							step	<= 1;
						end
					default:	step	<= 0;	// don't change anything
				endcase
			1:	// convert to BCD using the shift/add3 algorithm
				case (convStep)
					0:	// initialize conversion register
						begin
							freqINT		<= {24'h000000,freqBIN};
							counter		<= 0;
							convStep	<= 1;
						end
					1:	// check if add 3 needed for ALL six BCD digits
						begin
							if (freqINT[41:38]	>= 4'b0101)	freqINT[41:38]	<= freqINT[41:38] + 4'b0011;
							if (freqINT[37:34]	>= 4'b0101)	freqINT[37:34]	<= freqINT[37:34] + 4'b0011;
							if (freqINT[33:30]	>= 4'b0101)	freqINT[33:30]	<= freqINT[33:30] + 4'b0011;
							if (freqINT[29:26]	>= 4'b0101)	freqINT[29:26]	<= freqINT[29:26] + 4'b0011;
							if (freqINT[25:22]	>= 4'b0101)	freqINT[25:22]	<= freqINT[25:22] + 4'b0011;
							if (freqINT[21:18]	>= 4'b0101)	freqINT[21:18]	<= freqINT[21:18] + 4'b0011;
							convStep	<= 2;
						end
					2:	// shift by 1 into BCD part of register
						begin
							freqINT		<= freqINT	<< 1;
							counter		<= counter + 1;
							convStep	<= 3;
						end
					3:	// done?
						begin
							if (counter == 5'b10010)
								begin
									freqBCD		<= freqINT[41:18];
									convStep	<= 0;
									step		<= 2;
								end
							else	convStep	<= 1;
						end
					default:	
						begin
							convStep	<= 0;
							step		<= 0;
						end
				endcase
			2:	// wait some time before allowing next change
				begin
					if (waitCnt[23]	== 0)	waitCnt	<= waitCnt + 1;	// This restricts a manual change to once every 0.167 seconds
					else	
						begin
							waitCnt	<= 0;
							step	<= 0;
						end
				end
			default:	step	<= 0;
		endcase
	end

endmodule
