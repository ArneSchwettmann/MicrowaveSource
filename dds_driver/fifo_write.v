// Created by Steven Olmschenk (1) and revised by Isaiah Morgenstern (2)
// (1) Denison University, Olmschenk Research Group
// (2) University of Oklahoma, Schwettmann Research Group


// Module designed to perform a simple FIFO clear and write process during the USB communication protocol.

module	fifo_write(clk,
				   trigWrite,
				   clrFifo,
				   sclr,
				   writeReq);

input				clk;
input				trigWrite;		// indicates the write operation has been called
input				clrFifo;			// indicates whether the FIFO should be cleared

output			sclr;				// triggers a synchronous clear of the FIFO megafunction
output			writeReq;		// triggers a write operation to the FIFO megafunction

reg		[1:0]	status;			// status of FIFO write state machine
reg				prevTrigWrite;	// records previous trigger value
reg				sclr;				// reg for above output
reg				writeReq;		// reg for above output

always	@(posedge clk)
	case (status)
		2'b00:	// check if write has been called
			begin
				if (trigWrite != prevTrigWrite)
					begin
						if (clrFifo)	sclr	<= 1;
						prevTrigWrite			<= trigWrite;
						status					<= 2'b01;
					end
				else	status	<= 2'b00;	// if no FIFO write trigger, then do nothing
			end
		2'b01:	// end clear; trigger write
			begin
				sclr		<= 0;
				writeReq	<= 1;
				status		<= 2'b10;
			end
		2'b10:	// clean-up
			begin
				writeReq	<= 0;
				status		<= 2'b00;	// done writing to FIFO, so reinitialize
			end
		default:	status	<= 2'b00;
	endcase

endmodule
