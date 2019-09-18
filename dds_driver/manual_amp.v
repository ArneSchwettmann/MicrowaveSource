// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group

// Module Name:
// manual_amp

// Project Name:
// dds_driver

// Target Devices:
// DE0-CV Cyclone V FPGA board

// Notes:
// This module is designed to update the amplitude parameter for the DDS based on manual input by the user.  The user can 
// change the amplitude in approximate 1% increments.
//	The AD9954 has a 14-bit amplitude control word, allowing for an amplitude between 0 and 16383 to be set.  To make this
// a more user-friendly input, we will only do approximate 1% steps (i.e. 100 steps of magnitude 163).

module	manual_amp(clk,
				   trigup,
				   trigdown,
				   active,
				   ampDDS,
				   ampBCD);

input					clk;
input					trigup;		// triggers an amplitude increment by 1%
input					trigdown;	// triggers an amplitude decrement by 1%
input		[3:0]		active;		// sets status to update; 4'h2: indicates amplitude should be adjusted on trigger

output	[13:0]	ampDDS;		// output amplitude sent to the DDS (binary)
output	[11:0]	ampBCD;		// output amplitude sent to the LCD (BCD)

reg		[2:0]		counter;		// counter for the shift procedure
reg		[13:0]	ampDDS;		// register for the output above
reg		[11:0]	ampBCD;		// register for the output above
reg		[6:0]		ampBIN;		// the binary representation of the amplitude percentage (NOT the same binary number sent to the DDS)
reg		[18:0]	ampINT;		// the intermediate register that will be used to do the binary to BCD conversion.
reg		[2:0]		ampChange;	// determines how to change the amplitude (up or down)
reg		[2:0]		step;			// step in changing the amp
reg		[1:0]		convStep;	// step in converting from binary to BCD
reg		[23:0]	waitCnt;		// counter to wait some time between triggers

parameter	onePercent	= 163;
parameter	activeA		= 4'h2;

always @(posedge clk)
	begin
		ampChange	<= {(active == activeA), trigup, trigdown};
		
		case (step)
			0:	// look for an amp change trigger
				case (ampChange)
					3'b110:	// +1%
						begin
							if (ampDDS	<= (14'h3FAC - onePercent))	// check we are not at max amplitude
								begin
									ampDDS	<= ampDDS + onePercent;
									ampBIN	<= ampBIN + 1;
								end
							step	<= 1;
						end
					3'b101:	// -1%
						begin
							if (ampDDS	>= onePercent)	// check we are not at min amplitude
								begin
									ampDDS	<= ampDDS - onePercent;
									ampBIN	<= ampBIN - 1;
								end
							step	<= 1;
						end
					default:	step	<= 0;	// don't change anything
				endcase
			1:	// convert to BCD using the shift/add3 algorithm
				case (convStep)
					0:	// initialize conversion register
						begin
							ampINT		<= {12'h000,ampBIN};
							counter		<= 0;
							convStep	<= 1;
						end
					1:	// check if add 3 needed for ALL three BCD digits
						begin
							if (ampINT[18:15]	>= 4'b0101)	ampINT[18:15]	<= ampINT[18:15] + 4'b0011;
							if (ampINT[14:11]	>= 4'b0101)	ampINT[14:11]	<= ampINT[14:11] + 4'b0011;
							if (ampINT[10:7]	>= 4'b0101)	ampINT[10:7]	<= ampINT[10:7] + 4'b0011;
							convStep	<= 2;
						end
					2:	// shift by 1 into BCD part of register
						begin
							ampINT		<= ampINT	<< 1;
							counter		<= counter + 1;
							convStep	<= 3;
						end
					3:	// done?
						begin
							if (counter == 3'b111)
								begin
									ampBCD		<= ampINT[18:7];
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
