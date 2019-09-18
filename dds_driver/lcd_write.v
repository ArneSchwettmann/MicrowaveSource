// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group

// Module Name:
// lcd_write

// Project Name:
// dds_driver

// Target Devices:
// DE0-CV Cyclone V FPGA board


// Notes:
// Module that sends a data string to the LCD.

module lcd_write(clk,
				 recData,
				 send,
				 csb,
				 scl,
				 si);

input			clk;
input	[7:0]	recData;
input	[2:0]	send;

output				csb;		// drives CSB pin on LCD:  active low chip select signal
output				scl;		// drives SCL pin on LCD:  serial clock (less than 3 MHz)
output				si;		// drives SI  pin on LCD:  serial input data

reg					csb;		// output:  register for csb (above)
reg					scl;		// output:  register for scl (above)
reg					si;		// output:  register for si (above)
reg		[7:0]		data;		// stores received data (recData) at time of write initialization
reg		[31:0]	waitCnt;	// counts how many clock cycles we've waited for the scl tick-tock-tick
reg		[7:0]		status;	// status of the LCD write operation

parameter		waitTime	= 20000;	// number of clock cycles to give "ticks" for scl

always @(posedge clk)
	case (status)
		0:	// Look for write trigger
			case (send)
				3'b001:	// send instruction bits
					begin
						csb		<= 0;
						scl		<= 1;
						si		<= recData[7];	// get MSB of data ready
						data	<= recData;		// store all 8 bits of data
						waitCnt	<= 0;
						status	<= 1;
					end
				3'b011: // send data bits
					begin
						csb		<= 0;
						scl		<= 1;
						si		<= recData[7];	// get MSB of data ready
						data	<= recData;		// store all 8 bits of data
						waitCnt	<= 0;
						status	<= 1;
					end
				3'b101: // reset
					begin
						csb		<= 0;
						scl		<= 1;
						si		<= recData[7];
						data	<= recData;
						waitCnt	<= 0;
						status	<= 1;
					end
				3'b111: // reset
					begin
						csb		<= 0;
						scl		<= 1;
						si		<= recData[7];
						data	<= recData;
						waitCnt	<= 0;
						status	<= 1;
					end
				default:
					begin
						csb		<= 1;
						scl		<= 1;
						si		<= 0;
						waitCnt	<= 0;
						status	<= 4'h0;
					end
			endcase
		1:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 2;
					end	
			end
		2:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 3;
					end	
			end
		3:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 4;
					end	
			end
		4:	// bit 6
			begin
				si		<= data[6];
				status	<= 5;
			end
		5:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 6;
					end	
			end
		6:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 7;
					end	
			end
		7:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 8;
					end	
			end
		8:	// bit 5
			begin
				si		<= data[5];
				status	<= 9;
			end
		9:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 10;
					end	
			end
		10:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 11;
					end	
			end
		11:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 12;
					end	
			end
		12:	// bit 4
			begin
				si		<= data[4];
				status	<= 13;
			end
		13:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 14;
					end	
			end
		14:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 15;
					end	
			end
		15:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 16;
					end	
			end
		16:	// bit 3
			begin
				si		<= data[3];
				status	<= 17;
			end
		17:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 18;
					end	
			end
		18:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 19;
					end	
			end
		19:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 20;
					end	
			end
		20:	// bit 2
			begin
				si		<= data[2];
				status	<= 21;
			end
		21:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 22;
					end	
			end
		22:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 23;
					end	
			end
		23:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 24;
					end	
			end
		24:	// bit 1
			begin
				si		<= data[1];
				status	<= 25;
			end
		25:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 26;
					end	
			end
		26:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 27;
					end	
			end
		27:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 28;
					end	
			end
		28:	// bit 0
			begin
				si		<= data[0];
				status	<= 29;
			end
		29:	// scl (tick)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 30;
					end	
			end
		30:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 0;
						status	<= 31;
					end	
			end
		31:	// scl (tock)
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						scl		<= 1;
						status	<= 32;
					end	
			end
		32:	// clean-up
			begin
				if	(waitCnt	< waitTime)	waitCnt	<= waitCnt + 1;
				else
					begin
						waitCnt	<= 0;
						csb		<= 1;
						si		<= 0;
						status	<= 0;
					end	
			end
		default:
			begin
				csb		<= 1;
				scl		<= 1;
				si		<= 0;
				waitCnt	<= 0;
				status	<= 0;
			end
	endcase

endmodule
