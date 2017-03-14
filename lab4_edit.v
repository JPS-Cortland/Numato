`default_nettype none


///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- Template Toplevel Module (Modified)
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Original Author: Nathan Ickes, 6.111 staff
//
// with editing by Robert Sikora (with mentor John Sikora) 2016-2017
// to be compatible with the Numato Lab MimasV2 development board.
// This was Robert's senior project for a BSEE from Stony Brook University. 
// The following modules were added or heavily modified: 
//
//  - FilterControlModule (overall control, loosely based on the recorder module in the original lab4.v file)
//  - ToneVarModule (generates a variable pitch sine wave, based on the tone750 module) 
//  - MultiplierModule  (multiplies the input signal with a sine wave)
//
///////////////////////////////////////////////////////////////////////////////

module lab4(
  // Remove comment from any signals you use in your design!

  // AC97
  output wire /*beep,*/ audio_reset_b, ac97_synch, ac97_sdata_out,
  input wire ac97_bit_clock, ac97_sdata_in,
	
  input wire vol_button_down,
  input wire vol_button_up,
  input wire filt_button_down,
  input wire filt_button_up,
  input wire step_button_down,
  input wire step_button_up,
  output wire [7:0] led,

  output wire [7:0] IO_P9,  // monitors sent to the MimasV2 P9 connector.

  // CLOCK from the MimasV2 board

  input wire CLK_100MHz 
);
   ////////////////////////////////////////////////////////////////////////////
   //
   // Reset Generation
   //
   // A shift register primitive is used to generate an active-high reset
   // signal that remains high for 16 clock cycles after configuration finishes
   // and the FPGA's internal clocks begin toggling.
   //
   ////////////////////////////////////////////////////////////////////////////
   wire reset;
   SRL16 #(.INIT(16'hFFFF)) reset_sr(.D(1'b0), .CLK(CLK_100MHz), .Q(reset),
                                     .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1));
			    
   wire [7:0] from_ac97_data, to_ac97_data;
   wire ready;

   // AC97 driver
   lab4audio a(CLK_100MHz, reset, volume, from_ac97_data, to_ac97_data, ready,
	       audio_reset_b, ac97_sdata_out, ac97_sdata_in,
	       ac97_synch, ac97_bit_clock);

   //
   // FilterControlModule module selects which filter is in use, mostly through filt_num.
   // Volume can also be adjusted using up/down buttons.	
	// The variable step is used to set the frequency of the sine wave 
	// oscillator defined in ToneVarModule.
	//
   FilterControlModule filtcon(.clock(CLK_100MHz), .reset(reset), .ready(ready),
              .step(step), .filt_num(filt_num),
              .from_ac97_data(from_ac97_data),
				  .to_ac97_data(to_ac97_data));

   // output some signals to the Mimas V2  P9 connector for monitoring
   assign IO_P9[7] = ac97_bit_clock; 	// pin 1
   assign IO_P9[6] = audio_reset_b;  	// pin 2
   assign IO_P9[5] = ac97_sdata_out; 	// pin 3
   assign IO_P9[4] = ac97_sdata_in;		// pin 4
   assign IO_P9[3] = ac97_synch;			// pin 5
	assign IO_P9[2] = reset;   			// pin 6        
	assign IO_P9[1] = CLK_100MHz;			// pin 7


   // allow user to adjust volume up/down using buttons 5 (up) and 1 (down)
   wire vup,vdown;
   reg old_vup,old_vdown;
   debounce bup(.reset(reset),.clock(CLK_100MHz),.noisy(~vol_button_up),.clean(vup));
   debounce bdown(.reset(reset),.clock(CLK_100MHz),.noisy(~vol_button_down),.clean(vdown));
   reg [4:0] volume;
   always @ (posedge CLK_100MHz) begin
//      volume <= 5'd20;
     if (reset) volume <= 5'd15;
     else begin
	    if (vup & ~old_vup & volume != 5'd31) volume <= volume + 5'd1;       
	    if (vdown & ~old_vdown & volume != 5'd0) volume <= volume - 5'd1;       
     end
     old_vup <= vup;
     old_vdown <= vdown;
   end
	
//   allow user to select filter up/down using buttons 6 (up) and 2 (down)
   wire fup,fdown;
   reg old_fup,old_fdown;
   debounce fbup(.reset(reset),.clock(CLK_100MHz),.noisy(~filt_button_up),.clean(fup));
   debounce fbdown(.reset(reset),.clock(CLK_100MHz),.noisy(~filt_button_down),.clean(fdown));
   reg [3:0] filt_num;
   always @ (posedge CLK_100MHz) begin
     if (reset) filt_num <= 4'd0;
     else begin
	    if (fup & ~old_fup & filt_num != 4'd15)  filt_num <= filt_num + 4'h1;       
	    if (fdown & ~old_fdown & filt_num != 4'd0) filt_num <= filt_num - 4'h1;       
     end
     old_fup <= fup;
     old_fdown <= fdown;
   end

//   allow user to select step (frequency) up/down using buttons 4 (up) and 3 (down)
   wire sup,sdown;
   reg old_sup,old_sdown;
   debounce sbup(.reset(reset),.clock(CLK_100MHz),.noisy(~step_button_up),.clean(sup));
   debounce sbdown(.reset(reset),.clock(CLK_100MHz),.noisy(~step_button_down),.clean(sdown));
   reg [15:0] step;
   always @ (posedge CLK_100MHz) begin
     if (reset) step <= 16'h04B0;
     else begin
	    if (sup & ~old_sup & step != 16'hFFFF)  step <= step + 16'h0F;       
	    if (sdown & ~old_sdown & step != 16'h0) step <= step - 16'h0F; 		 
     end
     old_sup <= sup;
     old_sdown <= sdown;
   end

endmodule

///////////////////////////////////////////////////////////////////////////////
//
// FilterControlModule, based (loosely) on the recorder module
// of the original lab4.v file for the MIT Labkit
//
// In the original, two states could be selected by a single pushbutton
// In this modified version, the input filt_num (set by up/down buttons 
// in the calling module) determines which of several signals to send to
// the output to_ac97_data. 
///////////////////////////////////////////////////////////////////////////////

module FilterControlModule(
  input wire clock,	              // 100 MHz system clock
  input wire reset,                // 1 to reset to initial state
  input wire ready,                // 1 when AC97 data is available
  input wire [15:0]step,           // step in frequency
  input wire [3:0] filt_num,       // 0 unfiltered, 1 when using low-pass filter
  input wire [7:0] from_ac97_data, // 8-bit PCM data from mic
  output reg [7:0] to_ac97_data    // 8-bit PCM data to headphone
);  
   // test: playback 750hz tone, or loopback using incoming data
   wire [19:0] tone;
   tone750hz tone01(.clock(clock),.ready(ready),.pcm_data(tone));
	
	wire [19:0] tone_2;
	ToneVarModule tone02(.clock(clock),.ready(ready),.step(step),.pcm_data(tone_2));
	
	wire [19:0] tone_3;
	SquareVarModule tone03(.clock(clock),.ready(ready),.step(step),.pcm_data(tone_3));

	wire [17:0] filtered;
   FilterModule filt01(.clock(clock),.reset(reset),.ready(ready),.x(from_ac97_data), .y(filtered), .filt_num(filt_num));

   wire signed [15:0] mult;
   MultiplierModule mul(.clock(clock), .ready(ready), .x(from_ac97_data),.y(tone_2[19:12]),.prod (mult));

   wire signed [15:0] multsquare;
   MultiplierModule mulsq(.clock(clock), .ready(ready), .x(from_ac97_data),.y(tone_3[19:12]),.prod (multsquare));


   wire signed [17:0] delayed;
   DelayModule del(.clock(clock),.reset(reset),.ready(ready),.x(from_ac97_data), .y(delayed), .filt_num(filt_num));


   always @ (posedge clock) begin
      if (ready) begin
	 // get here when we've just received new data from the AC97
	 //to_ac97_data <= playback ? from_ac97_data : tone[19:12];
	 
	     case (filt_num[3:0])
	        4'h0:  to_ac97_data <= from_ac97_data;
		  	  4'h1:  to_ac97_data <= filtered[17:10]; // filtered #0; 
			  4'h2:  to_ac97_data <= filtered[17:10]; // filtered #1;
			  4'h3:  to_ac97_data <= tone_2[19:12];   // 
			  4'h4:  to_ac97_data <= mult[15:8];
			  4'h5:  to_ac97_data <= multsquare[15:8];
			  4'h6:  to_ac97_data <= delayed[17:10];   // echo with filt_num = 6
			  4'h7:  to_ac97_data <= delayed[17:10];   // reverb with filt_num = 7
           default: to_ac97_data <= from_ac97_data;
        endcase
      end
   end
endmodule


/////////////////////////////////////////
//      MultiplierModule()
//
//  Multiply the input signal times itself
///////////////////////////////////////////

module MultiplierModule(
   input wire clock,
	input wire ready,
	input wire signed [7:0] x,
	input wire signed [7:0] y,
	output reg signed [15:0] prod
);

   always @(posedge clock) begin
	   if(ready) 
		   prod <= x * y  ;			
	end
endmodule	

///////////////////////////////////////////////////////////////////////////////
//
// generate PCM data for 750hz sine wave (assuming f(ready) = 48khz)
//
///////////////////////////////////////////////////////////////////////////////

module tone750hz (
  input wire clock,
  input wire ready,
  output reg [19:0] pcm_data
);
   reg [8:0] index;

   initial begin
      index <= 8'h00;
      // synthesis attribute init of index is "00";
      pcm_data <= 20'h00000;
      // synthesis attribute init of pcm_data is "00000";
   end
   
   always @(posedge clock) begin
      if (ready) index <= index + 9'h1;
   end
   
   // one cycle of a sinewave in 64 20-bit samples
   always @(index) begin
      case (index[5:0])
        6'h00: pcm_data <= 20'h00000;
        6'h01: pcm_data <= 20'h0C8BD;
        6'h02: pcm_data <= 20'h18F8B;
        6'h03: pcm_data <= 20'h25280;
        6'h04: pcm_data <= 20'h30FBC;
        6'h05: pcm_data <= 20'h3C56B;
        6'h06: pcm_data <= 20'h471CE;
        6'h07: pcm_data <= 20'h5133C;
        6'h08: pcm_data <= 20'h5A827;
        6'h09: pcm_data <= 20'h62F20;
        6'h0A: pcm_data <= 20'h6A6D9;
        6'h0B: pcm_data <= 20'h70E2C;
        6'h0C: pcm_data <= 20'h7641A;
        6'h0D: pcm_data <= 20'h7A7D0;
        6'h0E: pcm_data <= 20'h7D8A5;
        6'h0F: pcm_data <= 20'h7F623;
        6'h10: pcm_data <= 20'h7FFFF;
        6'h11: pcm_data <= 20'h7F623;
        6'h12: pcm_data <= 20'h7D8A5;
        6'h13: pcm_data <= 20'h7A7D0;
        6'h14: pcm_data <= 20'h7641A;
        6'h15: pcm_data <= 20'h70E2C;
        6'h16: pcm_data <= 20'h6A6D9;
        6'h17: pcm_data <= 20'h62F20;
        6'h18: pcm_data <= 20'h5A827;
        6'h19: pcm_data <= 20'h5133C;
        6'h1A: pcm_data <= 20'h471CE;
        6'h1B: pcm_data <= 20'h3C56B;
        6'h1C: pcm_data <= 20'h30FBC;
        6'h1D: pcm_data <= 20'h25280;
        6'h1E: pcm_data <= 20'h18F8B;
        6'h1F: pcm_data <= 20'h0C8BD;
        6'h20: pcm_data <= 20'h00000;
        6'h21: pcm_data <= 20'hF3743;
        6'h22: pcm_data <= 20'hE7075;
        6'h23: pcm_data <= 20'hDAD80;
        6'h24: pcm_data <= 20'hCF044;
        6'h25: pcm_data <= 20'hC3A95;
        6'h26: pcm_data <= 20'hB8E32;
        6'h27: pcm_data <= 20'hAECC4;
        6'h28: pcm_data <= 20'hA57D9;
        6'h29: pcm_data <= 20'h9D0E0;
        6'h2A: pcm_data <= 20'h95927;
        6'h2B: pcm_data <= 20'h8F1D4;
        6'h2C: pcm_data <= 20'h89BE6;
        6'h2D: pcm_data <= 20'h85830;
        6'h2E: pcm_data <= 20'h8275B;
        6'h2F: pcm_data <= 20'h809DD;
        6'h30: pcm_data <= 20'h80000;
        6'h31: pcm_data <= 20'h809DD;
        6'h32: pcm_data <= 20'h8275B;
        6'h33: pcm_data <= 20'h85830;
        6'h34: pcm_data <= 20'h89BE6;
        6'h35: pcm_data <= 20'h8F1D4;
        6'h36: pcm_data <= 20'h95927;
        6'h37: pcm_data <= 20'h9D0E0;
        6'h38: pcm_data <= 20'hA57D9;
        6'h39: pcm_data <= 20'hAECC4;
        6'h3A: pcm_data <= 20'hB8E32;
        6'h3B: pcm_data <= 20'hC3A95;
        6'h3C: pcm_data <= 20'hCF044;
        6'h3D: pcm_data <= 20'hDAD80;
        6'h3E: pcm_data <= 20'hE7075;
        6'h3F: pcm_data <= 20'hF3743;
      endcase // case(index[5:0])
   end // always @ (index)
endmodule
///////////////////////////////////////////////////////////////////////////////
//
// generate PCM data for a variable frequency sine wave (assuming f(ready) = 48khz)
// Look up table of 64 values of 20 bits.
// The input value of step (16 bits) determines the frequency.
// Only the upper 6 bits of step are used giving 64 possible values for the location
// in the look up table.
// So a value of step = 1024 would require 64 increments of index for a full cycle.
// The output frequency would be 48 kHz/64 = 750 Hz.
// If step is 1, the frequency would be 1024 times slower or about 0.7 Hz 
///////////////////////////////////////////////////////////////////////////////
module ToneVarModule (
  input wire clock,
  input wire ready,
  input wire [15:0] step,
  output reg [19:0] pcm_data
);
   reg [15:0] index;

   initial begin
      index <= 8'h00;
      // synthesis attribute init of index is "00";
      pcm_data <= 20'h00000;
      // synthesis attribute init of pcm_data is "00000";
   end
   
   always @(posedge clock) begin
      if (ready) index <= index + step;
   end
   
   // one cycle of a sinewave in 64 20-bit samples
   always @(index) begin
      case (index[15:10])
        6'h00: pcm_data <= 20'h00000;
        6'h01: pcm_data <= 20'h0C8BD;
        6'h02: pcm_data <= 20'h18F8B;
        6'h03: pcm_data <= 20'h25280;
        6'h04: pcm_data <= 20'h30FBC;
        6'h05: pcm_data <= 20'h3C56B;
        6'h06: pcm_data <= 20'h471CE;
        6'h07: pcm_data <= 20'h5133C;
        6'h08: pcm_data <= 20'h5A827;
        6'h09: pcm_data <= 20'h62F20;
        6'h0A: pcm_data <= 20'h6A6D9;
        6'h0B: pcm_data <= 20'h70E2C;
        6'h0C: pcm_data <= 20'h7641A;
        6'h0D: pcm_data <= 20'h7A7D0;
        6'h0E: pcm_data <= 20'h7D8A5;
        6'h0F: pcm_data <= 20'h7F623;
        6'h10: pcm_data <= 20'h7FFFF;
        6'h11: pcm_data <= 20'h7F623;
        6'h12: pcm_data <= 20'h7D8A5;
        6'h13: pcm_data <= 20'h7A7D0;
        6'h14: pcm_data <= 20'h7641A;
        6'h15: pcm_data <= 20'h70E2C;
        6'h16: pcm_data <= 20'h6A6D9;
        6'h17: pcm_data <= 20'h62F20;
        6'h18: pcm_data <= 20'h5A827;
        6'h19: pcm_data <= 20'h5133C;
        6'h1A: pcm_data <= 20'h471CE;
        6'h1B: pcm_data <= 20'h3C56B;
        6'h1C: pcm_data <= 20'h30FBC;
        6'h1D: pcm_data <= 20'h25280;
        6'h1E: pcm_data <= 20'h18F8B;
        6'h1F: pcm_data <= 20'h0C8BD;
        6'h20: pcm_data <= 20'h00000;
        6'h21: pcm_data <= 20'hF3743;
        6'h22: pcm_data <= 20'hE7075;
        6'h23: pcm_data <= 20'hDAD80;
        6'h24: pcm_data <= 20'hCF044;
        6'h25: pcm_data <= 20'hC3A95;
        6'h26: pcm_data <= 20'hB8E32;
        6'h27: pcm_data <= 20'hAECC4;
        6'h28: pcm_data <= 20'hA57D9;
        6'h29: pcm_data <= 20'h9D0E0;
        6'h2A: pcm_data <= 20'h95927;
        6'h2B: pcm_data <= 20'h8F1D4;
        6'h2C: pcm_data <= 20'h89BE6;
        6'h2D: pcm_data <= 20'h85830;
        6'h2E: pcm_data <= 20'h8275B;
        6'h2F: pcm_data <= 20'h809DD;
        6'h30: pcm_data <= 20'h80000;
        6'h31: pcm_data <= 20'h809DD;
        6'h32: pcm_data <= 20'h8275B;
        6'h33: pcm_data <= 20'h85830;
        6'h34: pcm_data <= 20'h89BE6;
        6'h35: pcm_data <= 20'h8F1D4;
        6'h36: pcm_data <= 20'h95927;
        6'h37: pcm_data <= 20'h9D0E0;
        6'h38: pcm_data <= 20'hA57D9;
        6'h39: pcm_data <= 20'hAECC4;
        6'h3A: pcm_data <= 20'hB8E32;
        6'h3B: pcm_data <= 20'hC3A95;
        6'h3C: pcm_data <= 20'hCF044;
        6'h3D: pcm_data <= 20'hDAD80;
        6'h3E: pcm_data <= 20'hE7075;
        6'h3F: pcm_data <= 20'hF3743;
      endcase // case(index[5:0])
   end // always @ (index)
endmodule

///////////////////////////////////////////////////////////////////////////////
// SquareVarModule:
// Generate PCM data for a variable frequency square wave (assuming f(ready) = 48khz)
// Look up table of 64 values of 20 bits.
// The input value of step (16 bits) determines the frequency.
// Only the upper 6 bits of step are used giving 64 possible values for the location
// in the look up table.
// So a value of step = 1024 would require 64 increments of index for a full cycle.
// The output frequency would be 48 kHz/64 = 750 Hz.
// If step is 1, the frequency would be 1024 times slower or about 0.7 Hz 
///////////////////////////////////////////////////////////////////////////////
module SquareVarModule (
  input wire clock,
  input wire ready,
  input wire [15:0] step,
  output reg [19:0] pcm_data
);
   reg [15:0] index;

   initial begin
      index <= 8'h00;
      // synthesis attribute init of index is "00";
      pcm_data <= 20'h00000;
      // synthesis attribute init of pcm_data is "00000";
   end
   
   always @(posedge clock) begin
      if (ready) index <= index + step;
   end
   
   // one cycle of a square in 64 20-bit samples
   always @(index) begin
      case (index[15:10])
        6'h00: pcm_data <= 20'h3FFFF;
        6'h01: pcm_data <= 20'h3FFFF;
        6'h02: pcm_data <= 20'h3FFFF;
        6'h03: pcm_data <= 20'h3FFFF;
        6'h04: pcm_data <= 20'h3FFFF;
        6'h05: pcm_data <= 20'h3FFFF;
        6'h06: pcm_data <= 20'h3FFFF;
        6'h07: pcm_data <= 20'h3FFFF;
        6'h08: pcm_data <= 20'h3FFFF;
        6'h09: pcm_data <= 20'h3FFFF;
        6'h0A: pcm_data <= 20'h3FFFF;
        6'h0B: pcm_data <= 20'h3FFFF;
        6'h0C: pcm_data <= 20'h3FFFF;
        6'h0D: pcm_data <= 20'h3FFFF;
        6'h0E: pcm_data <= 20'h3FFFF;
        6'h0F: pcm_data <= 20'h3FFFF;
        6'h10: pcm_data <= 20'h3FFFF;
        6'h11: pcm_data <= 20'h3FFFF;
        6'h12: pcm_data <= 20'h3FFFF;
        6'h13: pcm_data <= 20'h3FFFF;
        6'h14: pcm_data <= 20'h3FFFF;
        6'h15: pcm_data <= 20'h3FFFF;
        6'h16: pcm_data <= 20'h3FFFF;
        6'h17: pcm_data <= 20'h3FFFF;
        6'h18: pcm_data <= 20'h3FFFF;
        6'h19: pcm_data <= 20'h3FFFF;
        6'h1A: pcm_data <= 20'h3FFFF;
        6'h1B: pcm_data <= 20'h3FFFF;
        6'h1C: pcm_data <= 20'h3FFFF;
        6'h1D: pcm_data <= 20'h3FFFF;
        6'h1E: pcm_data <= 20'h3FFFF;
        6'h1F: pcm_data <= 20'h3FFFF;
        6'h20: pcm_data <= 20'h00001;
        6'h21: pcm_data <= 20'h00001;
        6'h22: pcm_data <= 20'h00001;
        6'h23: pcm_data <= 20'h00000;
        6'h24: pcm_data <= 20'h00000;
        6'h25: pcm_data <= 20'h00000;
        6'h26: pcm_data <= 20'h00000;
        6'h27: pcm_data <= 20'h00000;
        6'h28: pcm_data <= 20'h00000;
        6'h29: pcm_data <= 20'h00000;
        6'h2A: pcm_data <= 20'h00000;
        6'h2B: pcm_data <= 20'h00000;
        6'h2C: pcm_data <= 20'h00000;
        6'h2D: pcm_data <= 20'h00000;
        6'h2E: pcm_data <= 20'h00000;
        6'h2F: pcm_data <= 20'h00000;
        6'h30: pcm_data <= 20'h00000;
        6'h31: pcm_data <= 20'h00000;
        6'h32: pcm_data <= 20'h00000;
        6'h33: pcm_data <= 20'h00000;
        6'h34: pcm_data <= 20'h00000;
        6'h35: pcm_data <= 20'h00000;
        6'h36: pcm_data <= 20'h00000;
        6'h37: pcm_data <= 20'h00000;
        6'h38: pcm_data <= 20'h00000;
        6'h39: pcm_data <= 20'h00000;
        6'h3A: pcm_data <= 20'h00000;
        6'h3B: pcm_data <= 20'h00000;
        6'h3C: pcm_data <= 20'h00000;
        6'h3D: pcm_data <= 20'h00000;
        6'h3E: pcm_data <= 20'h00000;
        6'h3F: pcm_data <= 20'h00000;
      endcase // case(index[5:0])
   end // always @ (index)
endmodule



///////////////////////////////////////////////////////////////////////////////
//
// Verilog equivalent to a BRAM, tools will infer the right thing!
// number of locations = 1<<LOGSIZE, width in bits = WIDTH.
// default is a 16K x 1 memory.
//
///////////////////////////////////////////////////////////////////////////////

module mybram #(parameter LOGSIZE=14, WIDTH=1)
              (input wire [LOGSIZE-1:0] addr,
               input wire clk,
               input wire [WIDTH-1:0] din,
               output reg [WIDTH-1:0] dout,
               input wire we);
   // let the tools infer the right number of BRAMs
   (* ram_style = "block" *)
   reg [WIDTH-1:0] mem[(1<<LOGSIZE)-1:0];
   always @(posedge clk) begin
     if (we) mem[addr] <= din;
     dout <= mem[addr];
   end
endmodule

///////////////////////////////////////////////////////////////////////////////
//
// Coefficients for a 31-tap low-pass FIR filter with Wn=.125 (eg, 3kHz for a
// 48kHz sample rate).  Since we're doing integer arithmetic, we've scaled
// the coefficients by 2**10
// Matlab command: round(fir1(30,.125)*1024)
//
///////////////////////////////////////////////////////////////////////////////

module coeffs31(
  input wire [3:0] select,
  input wire [4:0] index,
  output reg signed [9:0] coeff
);
  // tools will turn this into a 31x10 ROM
  always @(index or select)
     case (select[3:0])           //Which filter   
       4'd1: begin //select Filter 1 LOWPASS 3 kHz
         case (index[4:0])          // Which coefficient of filter 0
           5'd0:  coeff = -10'sd1;  // -10sd1;
           5'd1:  coeff = -10'sd1;  // -10sd1;
           5'd2:  coeff = -10'sd2;   //  -10sd3;
           5'd3:  coeff = -10'sd4;   //  -10sd5;
           5'd4:  coeff = -10'sd5;   //  -10sd6;
           5'd5:  coeff = -10'sd6;   //  -10sd7;
           5'd6:  coeff = -10'sd4;   //  -10sd5;
           5'd7:  coeff = -10'sd0;    //  -10sd0;
           5'd8:  coeff = 10'sd9;
           5'd9:  coeff = 10'sd22;
           5'd10: coeff = 10'sd39;
           5'd11: coeff = 10'sd57;
           5'd12: coeff = 10'sd76;
           5'd13: coeff = 10'sd91;  // 110  // I needed to reduce these values to avoid
           5'd14: coeff = 10'sd102;  // 123  // over-range distortion.
           5'd15: coeff = 10'sd105;  // 128
           5'd16: coeff = 10'sd102;  // 123
           5'd17: coeff = 10'sd91;  // 110
           5'd18: coeff = 10'sd76;
           5'd19: coeff = 10'sd57;
           5'd20: coeff = 10'sd39;
           5'd21: coeff = 10'sd22;
           5'd22: coeff = 10'sd9;
           5'd23: coeff = 10'sd0;
           5'd24: coeff = -10'sd4;
           5'd25: coeff = -10'sd6;
           5'd26: coeff = -10'sd5;
           5'd27: coeff = -10'sd4;
           5'd28: coeff = -10'sd2;
           5'd29: coeff = -10'sd1;
           5'd30: coeff = -10'sd1;
           default: coeff = 10'h0;
         endcase // End Filter 0
		 end
		 4'd2: begin  // select Filter 2 HIGHPASS 12kHz
         case (index[4:0])          // Which coefficient of filter 1
           5'd0:  coeff =  10'sd1;  // -10sd1;
           5'd1:  coeff =  10'sd0;  // -10sd1;
           5'd2:  coeff = -10'sd2;   //  -10sd3;
           5'd3:  coeff =  10'sd0;   //  -10sd5;
           5'd4:  coeff =  10'sd6;   //  -10sd6;
           5'd5:  coeff =  10'sd0;   //  -10sd7;
           5'd6:  coeff = -10'sd12;   //  -10sd5;
           5'd7:  coeff =  10'sd0;    //  -10sd0;
           5'd8: coeff =   10'sd23;
           5'd9: coeff =  -10'sd1;
           5'd10: coeff = -10'sd42;
           5'd11: coeff =  10'sd1;
           5'd12: coeff =  10'sd82;
           5'd13: coeff = -10'sd1;
           5'd14: coeff = -10'sd268;
           5'd15:  coeff = -10'sd300;  //!
           5'd16:  coeff = -10'sd268;
           5'd17: coeff = -10'sd1;
           5'd18: coeff =  10'sd82;
           5'd19: coeff =  10'sd1;
           5'd20: coeff = -10'sd42;  // 110  // I needed to reduce these values to avoid
           5'd21: coeff = -10'sd1;  // 123  // over-range distortion.
           5'd22: coeff =  10'sd23;  // 128
           5'd23: coeff =  10'sd0;  // 123
           5'd24: coeff = -10'sd12;  // 110
           5'd25: coeff =  10'sd0;
           5'd26: coeff =  10'sd6;
           5'd27: coeff =  10'sd0;
           5'd28: coeff = -10'sd2;
           5'd29: coeff =  10'sd0;
           5'd30: coeff =  10'sd1;
           default: coeff = 10'h0;
         endcase  // End Filter 1
		 end
		 default: coeff = 10'h0;  // didn't find selected filter ( should be 0 or 1)
     endcase
		 
endmodule



///////////////////////////////////////////////////////////////////////////////
//
// 
//
///////////////////////////////////////////////////////////////////////////////

module delay4k(
  input wire [3:0] index,
  output reg [11:0] delay
);
  // tools will turn this into a 16 x 12bit ROM
  always @(index)
 
     case (index[3:0])          
           4'd0:  delay =  12'd4000; 
           4'd1:  delay =  12'd3477;  
           4'd2:  delay =  12'd2567;   
           4'd3:  delay =  12'd1823;   
           4'd4:  delay =  12'd1;   
           4'd5:  delay =  12'd1;   
           4'd6:  delay =  12'd1;   
           4'd7:  delay =  12'd1;   
           4'd8:  delay =  12'd1;
           4'd9:  delay =  12'd1;
           4'd10: delay =  12'd1;
           4'd11: delay =  12'd1;
           4'd12: delay =  12'd1;
           4'd13: delay =  12'd1;  
           4'd14: delay =  12'd1;  
           4'd15: delay =  12'd1;    
           default: delay = 12'd1;
      endcase 
	 
endmodule



///////////////////////////////////////////////////////////////////////////////
//
// Switch Debounce Module
//
///////////////////////////////////////////////////////////////////////////////

module debounce (
  input wire reset, clock, noisy,
  output reg clean
);
  reg [18:0] count;
  reg new;

  always @(posedge clock)
    if (reset) begin
      count <= 0;
      new <= noisy;
      clean <= noisy;
    end
    else if (noisy != new) begin
      // noisy input changed, restart the .01 sec clock
      new <= noisy;
      count <= 0;
    end
    else if (count == 270000)
      // noisy input stable for .01 secs, pass it along!
      clean <= new;
    else
      // waiting for .01 sec to pass
      count <= count + 1'b1;

endmodule

//////////////////////////////////////////////////////////////////////////////
//                         FilterModule()
///////////////////////////////////////////////////////////////////////////////
//
// 31-tap FIR filter, 8-bit signed data, 10-bit signed coefficients.
// ready is asserted whenever there is a new sample on the X input,
// the Y output should also be sampled at the same time.  Assumes at
// least 32 clocks between ready assertions.  Note that since the
// coefficients have been scaled by 2**10, so has the output (it's
// expanded from 8 bits to 18 bits).  To get an 8-bit result from the
// filter just divide by 2**10, ie, use Y[17:10].
//
// Only one multiplication on each clock cycle.
//
///////////////////////////////////////////////////////////////////////////////

module FilterModule(
  input wire clock,reset,ready,
  input wire signed [7:0] x,
  output wire signed [17:0] y,
  input wire [3:0] filt_num
);

  reg [4:0]  offset;   // offset to location of current audio data in circular buffer.
  reg [4:0]  index;    // index to filter coefficients. Range 0 - 30.
  reg        done;     // 
  reg signed [7:0]  sample[0:31];
  reg signed [17:0] accum;
  reg signed [17:0] output_reg;
  wire signed [9:0] coeff; 
 
   assign y    = output_reg;   // output_reg;
	
   initial begin
     index     <= 5'h0;
     accum     <= 18'd0;
     offset    <= 5'h0;
	  output_reg <= 18'd0;
	  	  
	  sample[0] <= 0;
	  sample[1] <= 0;
	  sample[2] <= 0;
	  sample[3] <= 0;
	  sample[4] <= 0;
	  sample[5] <= 0;
	  sample[6] <= 0;
	  sample[7] <= 0;
	  sample[8] <= 0;
	  sample[9] <= 0;
	  sample[10] <= 0;
	  sample[11] <= 0;
	  sample[12] <= 0;
	  sample[13] <= 0;
	  sample[14] <= 0;
	  sample[15] <= 0;
	  sample[16] <= 0;
	  sample[17] <= 0;
	  sample[18] <= 0;
	  sample[19] <= 0;
	  sample[20] <= 0;
	  sample[21] <= 0;
	  sample[22] <= 0;
	  sample[23] <= 0;
	  sample[24] <= 0;
	  sample[25] <= 0;
	  sample[26] <= 0;
	  sample[27] <= 0;
	  sample[28] <= 0;
	  sample[29] <= 0;
	  sample[30] <= 0;
	  sample[31] <= 0;
  end

//  Increment the offset once on each (audio sample) ready.

  always @(posedge clock) begin   // could this increment more than once on each "ready" ?
     if (ready) begin
         offset <= offset + 5'h1;
     end			
  end

  coeffs31 coe(filt_num, index,coeff);

  always @(posedge clock) begin
    if(ready) begin
	    sample[offset] <= x;	
		 index  <= 0;		
       accum  <= 0;
       done   <= 0;		 
	 end
	 
    if (!ready && (index < 30) ) begin   // calculate when ready is low. (< 30)
	    accum <= accum + coeff * sample[offset-index];
	    index <= index + 5'h1;
    end		
    if (!done && (index == 30) ) begin   // when finished, update output (== 30)
		 output_reg <= accum;   // update ouput register, once.
		 done <= 1'b1;           
    end	

  end
  
endmodule

//////////////////////////////////////////////////////////////////////////////
//                         DelayModule()
///////////////////////////////////////////////////////////////////////////////
//
// 
// Stores audio input samples in an 8 bit x 4k long array and will output the 
// sum of the set of samples[m] returned by an instance of the module delay4k.
// For example, this could be the most recent sample plus the sample from
// 4000 samples ago. This would give an echo from 1/48kHz * 4000 seconds ago. 
// Or about 80 ms.
///////////////////////////////////////////////////////////////////////////////

module DelayModule(
  input wire clock,reset,ready,
  input wire signed [7:0] x,
  output wire signed [17:0] y,
  input wire [3:0] filt_num
);

  reg [11:0]  offset;   // offset to location of current audio data in circular buffer.
  reg [3:0]  index;    // index to case statement
  reg         done;     // 

  reg signed [7:0]  sample[0:4096];
  reg signed [17:0] accum;
  reg signed [17:0] output_reg;
  wire       [11:0] delay; 
  reg        [7:0]  size;
  reg signed [9:0]  revamp;
  
 
   assign y    = output_reg;   // output_reg;

//	assign y = {x, 10'h00};     // for straight through
   initial begin
	  offset    <= 12'h0;
     index     <= 4'h0;
     accum     <= 18'd0;
	  output_reg <= 18'd0;	  	  
	  sample[0] <= 0;

  end

 

// This module demonstrates both a single echo (filt_num = 6)
// and reverberation (filt_num = 7).
// For echo, use only the first value of delay (size = 1)and increase the 
// amplitude of that echo so that it is very distinct (revamp = 1FF).
// For reverberation, four delays are used. Since the samples from
// these different delays are all added, the scale factor needs
// to be smaller, otherwise the amplitude could go beyond a valid range. (revamp = 0AF)
//

  always @(posedge clock) begin   // could this increment more than once on each "ready" ?
     if (ready) begin
        offset <= offset + 12'h1;     //  Increment the offset once on each ready (when new audio sample) 
        if (filt_num == 6) begin
           size   <= 8'd1;
           revamp <= 10'sh1FF;
        end		  
		  else begin
		     size <= 8'd4;
			  revamp <= 10'sh0BF;
		  end

     end			
  end



  delay4k del(index,delay);

  always @(posedge clock) begin
    if(ready) begin	
		 index  <= 0;		
       accum  <= 10'sh1FF * x;   // 1FF put the current sample into the accumulator
       done   <= 0;		 
	 end

    if (!ready && (index < size) ) begin   // calculate when ready is low. (< 2)
	    accum <= accum + revamp * sample[offset - delay];
	    index <= index + 4'd1;
    end		
    if (!done && (index == size) ) begin         // when finished, update output (== 2)

       if(filt_num == 6 ) begin
  		    sample[offset] <=   x;               // for echo                		 
       end
		 else begin
		    sample[offset] <=   accum[17:10];     // for reverberation
       end

	    output_reg     <=  accum;
		 done <= 1;           
    end	

  end
  
endmodule

///////////////////////////////////////////////////////////////////////////////
//
// bi-directional monaural interface to AC97
// Original from the lab4.v file for the MIT 6.111 FPGA Labkit
// with minor modifications for the Numato Lab MimasV2 FPGA Development Board.
///////////////////////////////////////////////////////////////////////////////

module lab4audio (
  input wire CLK_100MHz,
  input wire reset,
  input wire [4:0] volume,
  output wire [7:0] audio_in_data,
  input wire [7:0] audio_out_data,
  output wire ready,
  output reg audio_reset_b,   // ac97 interface signals
  output wire ac97_sdata_out,
  input wire ac97_sdata_in,
  output wire ac97_synch,
  input wire ac97_bit_clock
);

  wire [7:0] command_address;
  wire [15:0] command_data;
  wire command_valid;
  wire [19:0] left_in_data, right_in_data;
  wire [19:0] left_out_data, right_out_data;

  // wait a little before enabling the AC97 codec
  reg [9:0] reset_count;
  always @(posedge CLK_100MHz) begin
    if (reset) begin
      audio_reset_b = 1'b0;
      reset_count = 0;
    end else if (reset_count == 1023)
      audio_reset_b = 1'b1;
    else
      reset_count = reset_count + 10'h1;
  end

  wire ac97_ready;
  ac97 ac97(.ready(ac97_ready),
            .command_address(command_address),
            .command_data(command_data),
            .command_valid(command_valid),
            .left_data(left_out_data), .left_valid(1'b1),
            .right_data(right_out_data), .right_valid(1'b1),
            .left_in_data(left_in_data), .right_in_data(right_in_data),
            .ac97_sdata_out(ac97_sdata_out),
            .ac97_sdata_in(ac97_sdata_in),
            .ac97_synch(ac97_synch),
            .ac97_bit_clock(ac97_bit_clock));

  // ready: one cycle pulse synchronous with CLK_100MHz
  reg [2:0] ready_sync;
  always @ (posedge CLK_100MHz) ready_sync <= {ready_sync[1:0], ac97_ready};
  assign ready = ready_sync[1] & ~ready_sync[2];

  reg [7:0] out_data;
  always @ (posedge CLK_100MHz)
    if (ready) out_data <= audio_out_data;
  assign audio_in_data = left_in_data[19:12];
  assign left_out_data = {out_data, 12'b000000000000};
  assign right_out_data = left_out_data;

  // generate repeating sequence of read/writes to AC97 registers
  ac97commands cmds(.clock(CLK_100MHz), .ready(ready),
                    .command_address(command_address),
                    .command_data(command_data),
                    .command_valid(command_valid),
                    .volume(volume),
                    .source(3'b100));     // 3'b000 mic, 3'b100 line in
endmodule

// assemble/disassemble AC97 serial frames
module ac97 (
  output reg ready,
  input wire [7:0] command_address,
  input wire [15:0] command_data,
  input wire command_valid,
  input wire [19:0] left_data,
  input wire left_valid,
  input wire [19:0] right_data,
  input wire right_valid,
  output reg [19:0] left_in_data, right_in_data,
  output reg ac97_sdata_out,
  input wire ac97_sdata_in,
  output reg ac97_synch,
  input wire ac97_bit_clock
);
  reg [7:0] bit_count;

  reg [19:0] l_cmd_addr;
  reg [19:0] l_cmd_data;
  reg [19:0] l_left_data, l_right_data;
  reg l_cmd_v, l_left_v, l_right_v;

  initial begin
    ready <= 1'b0;
    // synthesis attribute init of ready is "0";
    ac97_sdata_out <= 1'b0;
    // synthesis attribute init of ac97_sdata_out is "0";
    ac97_synch <= 1'b0;
    // synthesis attribute init of ac97_synch is "0";

    bit_count <= 8'h00;
    // synthesis attribute init of bit_count is "0000";
    l_cmd_v <= 1'b0;
    // synthesis attribute init of l_cmd_v is "0";
    l_left_v <= 1'b0;
    // synthesis attribute init of l_left_v is "0";
    l_right_v <= 1'b0;
    // synthesis attribute init of l_right_v is "0";

    left_in_data <= 20'h00000;
    // synthesis attribute init of left_in_data is "00000";
    right_in_data <= 20'h00000;
    // synthesis attribute init of right_in_data is "00000";
  end

  always @(posedge ac97_bit_clock) begin
    // Generate the sync signal
    if (bit_count == 255)
      ac97_synch <= 1'b1;
    if (bit_count == 15)
      ac97_synch <= 1'b0;

    // Generate the ready signal
    if (bit_count == 128)
      ready <= 1'b1;
    if (bit_count == 2)
      ready <= 1'b0;

    // Latch user data at the end of each frame. This ensures that the
    // first frame after reset will be empty.
    if (bit_count == 255) begin
      l_cmd_addr <= {command_address, 12'h000};
      l_cmd_data <= {command_data, 4'h0};
      l_cmd_v <= command_valid;
      l_left_data <= left_data;
      l_left_v <= left_valid;
      l_right_data <= right_data;
      l_right_v <= right_valid;
    end

    if ((bit_count >= 0) && (bit_count <= 15))
      // Slot 0: Tags
      case (bit_count[3:0])
        4'h0: ac97_sdata_out <= 1'b1;      // Frame valid
        4'h1: ac97_sdata_out <= l_cmd_v;   // Command address valid
        4'h2: ac97_sdata_out <= l_cmd_v;   // Command data valid
        4'h3: ac97_sdata_out <= l_left_v;  // Left data valid
        4'h4: ac97_sdata_out <= l_right_v; // Right data valid
        default: ac97_sdata_out <= 1'b0;
      endcase
    else if ((bit_count >= 16) && (bit_count <= 35))
      // Slot 1: Command address (8-bits, left justified)
      ac97_sdata_out <= l_cmd_v ? l_cmd_addr[35-bit_count] : 1'b0;
    else if ((bit_count >= 36) && (bit_count <= 55))
      // Slot 2: Command data (16-bits, left justified)
      ac97_sdata_out <= l_cmd_v ? l_cmd_data[55-bit_count] : 1'b0;
    else if ((bit_count >= 56) && (bit_count <= 75)) begin
      // Slot 3: Left channel
      ac97_sdata_out <= l_left_v ? l_left_data[19] : 1'b0;
      l_left_data <= { l_left_data[18:0], l_left_data[19] };
    end
    else if ((bit_count >= 76) && (bit_count <= 95))
      // Slot 4: Right channel
      ac97_sdata_out <= l_right_v ? l_right_data[95-bit_count] : 1'b0;
    else
      ac97_sdata_out <= 1'b0;

    bit_count <= bit_count + 8'h1;
  end // always @ (posedge ac97_bit_clock)

  always @(negedge ac97_bit_clock) begin
    if ((bit_count >= 57) && (bit_count <= 76))
      // Slot 3: Left channel
      left_in_data <= { left_in_data[18:0], ac97_sdata_in };
    else if ((bit_count >= 77) && (bit_count <= 96))
      // Slot 4: Right channel
      right_in_data <= { right_in_data[18:0], ac97_sdata_in };
  end
endmodule

// issue initialization commands to AC97
module ac97commands (
  input wire clock,
  input wire ready,
  output wire [7:0] command_address,
  output wire [15:0] command_data,
  output reg command_valid,
  input wire [4:0] volume,
  input wire [2:0] source
);
  reg [23:0] command;

  reg [3:0] state;
  initial begin
    command <= 4'h0;
    // synthesis attribute init of command is "0";
    command_valid <= 1'b0;
    // synthesis attribute init of command_valid is "0";
    state <= 16'h0000;
    // synthesis attribute init of state is "0000";
  end

  assign command_address = command[23:16];
  assign command_data = command[15:0];

  wire [4:0] vol;
  assign vol = 5'd31-volume;  // convert to attenuation

  always @(posedge clock) begin
    if (ready) state <= state + 4'h1;

    case (state)
      4'h0: // Read ID
        begin
          command <= 24'h80_0000;
          command_valid <= 1'b1;
        end
      4'h1: // Read ID
        command <= 24'h80_0000;
      4'h3: // headphone volume
        command <= { 8'h04, 3'b000, vol, 3'b000, vol };
      4'h5: // PCM volume
        command <= 24'h18_0808;
      4'h6: // Record source select
        command <= { 8'h1A, 5'b00000, source, 5'b00000, source};
      4'h7: // Record gain = max
        command <= 24'h1C_0F0F;
      4'h9: // set +20db mic gain
        command <= 24'h0E_8048;
      4'hA: // Set beep volume
        command <= 24'h0A_0000;
      4'hB: // PCM out bypass mix1
        command <= 24'h20_8000;
      default:
        command <= 24'h80_0000;
    endcase // case(state)
  end // always @ (posedge clock)
endmodule // ac97commands
