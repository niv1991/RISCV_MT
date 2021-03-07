// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Francesco Conti - f.conti@unibo.it                         //
//                                                                            //
// Additional contributions by:                                               //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    RISC-V register file                                       //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Register file with 31x 32 bit wide registers. Register 0   //
//                 is fixed to 0. This register file is based on flip-flops.  //
//                 Also supports the fp-register file now if FPU=1            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
import riscv_defines::THREAD_ADDR_WIDTH;
import riscv_defines::NUM_THREADS;
module riscv_register_file
#(
    parameter ADDR_WIDTH    = 5,
    parameter DATA_WIDTH    = 32,
    parameter FPU           = 0
)
(
    // Clock and Reset
    input  logic         clk,
    input  logic         rst_n,
    input  logic         test_en_i,

    // Threads
    input  logic [THREAD_ADDR_WIDTH-1:0] hart_id_i,    // Read
    input  logic [THREAD_ADDR_WIDTH-1:0] hart_id_ex_i, // Write Port B
    input  logic [THREAD_ADDR_WIDTH-1:0] hart_id_wb_i, // Write Port A

    //Read port R1
    input  logic [ADDR_WIDTH-1:0]  raddr_a_i,
    output logic [DATA_WIDTH-1:0]  rdata_a_o,

    //Read port R2
    input  logic [ADDR_WIDTH-1:0]  raddr_b_i,
    output logic [DATA_WIDTH-1:0]  rdata_b_o,

    //Read port R3
    input  logic [ADDR_WIDTH-1:0]  raddr_c_i,
    output logic [DATA_WIDTH-1:0]  rdata_c_o,

    // Write port W1
    input logic [ADDR_WIDTH-1:0]   waddr_a_i,
    input logic [DATA_WIDTH-1:0]   wdata_a_i,
    input logic                    we_a_i,

    // Write port W2
    input logic [ADDR_WIDTH-1:0]   waddr_b_i,
    input logic [DATA_WIDTH-1:0]   wdata_b_i,
    input logic                    we_b_i
);

  // number of integer registers // number of words PER THREAD
  localparam    NUM_WORDS     = 2**(ADDR_WIDTH) - 1; // -1 for mem0.
  // number of floating point registers
  localparam    NUM_FP_WORDS  = 2**(ADDR_WIDTH);
  localparam    NUM_TOT_WORDS = FPU ? NUM_WORDS + NUM_FP_WORDS : NUM_WORDS; // NUM_WORDS
  
  // R0
  logic [DATA_WIDTH-1:0]                                  mem0;
  // integer register file
  logic [NUM_THREADS-1:0][NUM_WORDS-1:0][DATA_WIDTH-1:0]  mem;

  // fp register file
  logic [NUM_FP_WORDS-1:0][DATA_WIDTH-1:0]                mem_fp;

  // write enable signals for all registers
  logic [NUM_TOT_WORDS-1:0]                               we_a_dec;
  logic [NUM_TOT_WORDS-1:0]                               we_b_dec;
  

   //-----------------------------------------------------------------------------
   //-- READ : Read address decoder RAD
   //-----------------------------------------------------------------------------
generate  // added by Udi 28/Feb/2019 required by Quartus
   if (FPU == 1) begin : fpu1 // label added by Udi 28/Feb/2019 required by Quartus
      assign rdata_a_o = raddr_a_i[5] ? mem_fp[raddr_a_i[4:0]] : (raddr_a_i[4:0] > 0) ? mem[hart_id_i][raddr_a_i[4:0]-5'd1] : mem0;
      assign rdata_b_o = raddr_b_i[5] ? mem_fp[raddr_b_i[4:0]] : (raddr_b_i[4:0] > 0) ? mem[hart_id_i][raddr_b_i[4:0]-5'd1] : mem0;
      assign rdata_c_o = raddr_c_i[5] ? mem_fp[raddr_c_i[4:0]] : (raddr_c_i[4:0] > 0) ? mem[hart_id_i][raddr_c_i[4:0]-5'd1] : mem0;
   end else begin : fpu0 // label added by Udi 28/Feb/2019 required by Quartus
      assign rdata_a_o = (raddr_a_i[4:0] > 0) ? mem[hart_id_i][raddr_a_i[4:0]-5'd1] : mem0; 
      assign rdata_b_o = (raddr_b_i[4:0] > 0) ? mem[hart_id_i][raddr_b_i[4:0]-5'd1] : mem0;
      assign rdata_c_o = (raddr_c_i[4:0] > 0) ? mem[hart_id_i][raddr_c_i[4:0]-5'd1] : mem0;
   end
endgenerate 

  //-----------------------------------------------------------------------------
  //-- WRITE : Write Address Decoder (WAD), combinatorial process
  //-----------------------------------------------------------------------------
  always_comb
  begin : we_a_decoder
    for (int i = 1; i <= NUM_TOT_WORDS; i++) begin
      if (waddr_a_i == i)
        we_a_dec[i-1] = we_a_i; // i-1 to skip 0 [0->30]
      else
        we_a_dec[i-1] = 1'b0;
    end
  end

  always_comb
  begin : we_b_decoder
    for (int i = 1; i <= NUM_TOT_WORDS; i++) begin
      if (waddr_b_i == i)
        we_b_dec[i-1] = we_b_i;
      else
        we_b_dec[i-1] = 1'b0;
    end
  end

  genvar i,l;
  generate
   //-----------------------------------------------------------------------------
   //-- WRITE : Write operation
   //-----------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
      if(~rst_n) begin
        // R0 is nil
        mem0 <= {DATA_WIDTH{1'b0}}; // No need to duplicate R0
      end else begin
        // R0 is nil
        mem0 <= {DATA_WIDTH{1'b0}};
      end
    end

    // R0 is nil
    // loop from 1 to NUM_WORDS-1 as R0 is nil --> changed to --> loop from 0 to NUM_WORDS-1 (NUM_WORDS is smaller by 1 as R0 was moved out)
    for (i = 0; i < NUM_WORDS; i++)
    begin : rf_gen
      always_ff @(posedge clk, negedge rst_n)
      begin : register_write_behavioral
        if (rst_n==1'b0)  
          for(int j=0;j<NUM_THREADS;j++)  
            mem[j][i] <= {DATA_WIDTH{1'b0}};
        else begin
          /* 
           * Thread IDs must not be the same as it clearly creates a conflict.
           * (If threads are stalled for example)
           * Note: hart_id_wb_i is 'x' until it is required for the first time.
           */

          if(we_b_dec[i] == 1'b1)
            mem[hart_id_ex_i][i] <= wdata_b_i;
          /* 
			RF refisters are offset by 1 when multithreading is enabled.
		  */
          if(we_a_dec[i] == 1'b1)
            mem[hart_id_wb_i][i] <= wdata_a_i;
        end
      end

    end
    

     if (FPU == 1) begin
        // Floating point registers 
        for(l = 0; l < NUM_FP_WORDS; l++) 
        begin : fpu1 // label added by Udi 28/Feb/2019 required by Quartus

          always_ff @(posedge clk, negedge rst_n)
          begin : fp_regs
              if (rst_n==1'b0)
                mem_fp[l] <= '0;
              else if(we_b_dec[l+NUM_WORDS] == 1'b1)
                mem_fp[l] <= wdata_b_i;
              else if(we_a_dec[l+NUM_WORDS] == 1'b1)
                mem_fp[l] <= wdata_a_i;
          end

        end
     end
  endgenerate

endmodule
