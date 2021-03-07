//////////////////////////////////////////////////////////////
//                      SIMPLE_ARBITER.sv                   //
//                        Michael Pines                     //
//////////////////////////////////////////////////////////////
//           Assume incoming requests are held until        //
//                     granted (may be longer).             //
import riscv_defines::THREAD_ADDR_WIDTH;
import riscv_defines::NUM_THREADS;
`define LD_FLAG 0
`define ST_FLAG 1
`define J_FLAG 2
`define BR_FLAG 3
module simple_arbiter
(
	input  logic                         clk,
	input  logic                         rst,
	input  logic 						 enable,

    input  logic [NUM_THREADS-1:0]       request,
	// Pipeline granted, output thread index.
	output logic [THREAD_ADDR_WIDTH-1:0] gnt, // Dispatch
	output logic [THREAD_ADDR_WIDTH-1:0] nxt, // Fetch
	// NEW
	input  logic [THREAD_ADDR_WIDTH-1:0] hart_wb_i,
	input  logic [THREAD_ADDR_WIDTH-1:0] hart_ex_i,
	input  logic [THREAD_ADDR_WIDTH-1:0] hart_id_i,

	input  logic 						 data_we_i,
	input  logic 						 data_req_i,
	input  logic 						 data_gnt_i,
	input  logic 						 data_rvalid_i,
	input  logic 						 branch_sig_i,
	input  logic 						 jump_sig_i,

	input  logic [31:0]                  jump_target_i,
	input  logic [31:0]                  branch_target_i,

	input  logic [31:0] 				 pc_if_i,
	input  logic [31:0] 				 pc_id_i,

	input  logic [31:0] 				 instr_i,
	input  logic [31:0] 				 data_i
); 

parameter  MASTER_THREAD  = {THREAD_ADDR_WIDTH{1'b0}};
localparam ONEADDR = {{THREAD_ADDR_WIDTH-1{1'b0}},1'b1};
// NEW //
// Define thread entity 
typedef struct packed {
	logic [31:0] pc;
	logic [31:0] instruction;
	logic [31:0] load_resault;
	logic [3:0]  flags;
	/*
	logic 		 ld_flag; // load, need to wait
	logic 		 st_flag; // store, can probably move on
	logic 		 j_flag;  // jump
	logic 		 br_flag; // branch
	*/
	// logic 		 ex_flag; // exception
	// logic 		 irq_flag; // interrupt
} thread;
thread [NUM_THREADS-1:0] threads;

logic [NUM_THREADS-1:0] thread_status;

assign hart_in_id = threads[hart_id_i];
assign hart_in_ex = threads[hart_ex_i];
assign hart_in_wb = threads[hart_wb_i];
always@(posedge clk, negedge rst)
begin
	if (rst==1'b0)  
	begin
		threads <= '{default: '0};
	end else 
	begin
		threads[hart_id_i].instruction <= instr_i;
		
		for(int j = 0; j < NUM_THREADS; j++)
			threads[j].flags[1:3] <= 3'd0;
		// JUMPS
		if(~jump_sig_i) 
			threads[hart_id_i].pc <= pc_id_i;
		else begin
			threads[hart_id_i].pc <= jump_target_i;
			threads[hart_id_i].flags[`J_FLAG] <= 1'b1;
			$display("[%0t] ARBITER HART_%0d IN ID : JUMP !!!!",$time,hart_id_i);
		end
		// BRANCHES
		if(branch_sig_i) begin
			threads[hart_ex_i].pc <= branch_target_i;
			threads[hart_ex_i].flags[`BR_FLAG] <= 1'b1;
			thread_status[hart_ex_i] <= 1'b0;
			$display("[%0t] ARBITER HART_%0d IN EX : BRANCH !!!!",$time,hart_ex_i);
		end else thread_status[hart_ex_i] <= 1'b1;
		// LOADS
		if(threads[hart_wb_i].ld_flag == 1'b0) 
		begin
			threads[hart_wb_i].flags[`LD_FLAG] <= (data_req_i & data_gnt_i & ~data_we_i);
			if(data_req_i & data_gnt_i & ~data_we_i) begin
				thread_status[hart_wb_i] <= 1'b0;
				$display("[%0t] + ARBITER HART_%0d IN WB : LOAD FLAG SET !!!!",$time,hart_wb_i);
			end
		end
		else if(data_rvalid_i) 
		begin
			threads[hart_wb_i].flags[`LD_FLAG] <= 1'b0;
			threads[hart_wb_i].load_resault <= data_i;
			thread_status[hart_wb_i] <= 1'b1;
			$display("[%0t] - ARBITER HART_%0d IN WB : LOAD FLAG CLEARED !!!!\n\tREQUESTED DATA = 0x%h",$time,hart_wb_i,data_i);
		end 
		else if((hart_wb_i != hart_ex_i) && (hart_wb_i != hart_id_i)) 
			thread_status[hart_wb_i] <= 1'b1;
	end
end
// NEW //
always@(posedge clk, negedge rst)
begin
    // Output master thread when reset
	if (rst==1'b0)  begin
		gnt <= MASTER_THREAD;
		//nxt <= MASTER_THREAD;
	end
	else if(enable)
    begin
        // Output
		gnt <= gnt;
		for(int i = 1; i < NUM_THREADS; i++) // Not requesting if stalled
			if(request[gnt + i[THREAD_ADDR_WIDTH-1:0]]) 
			begin
				gnt <= gnt + i[THREAD_ADDR_WIDTH-1:0];
				break;
			end
	end
end

always_comb
begin
		nxt = gnt;

		for(int j = 1; j < NUM_THREADS; j++)
				if(request[gnt + j[THREAD_ADDR_WIDTH-1:0]]) 
				begin
					nxt = gnt + j[THREAD_ADDR_WIDTH-1:0];
					break;
				end
	
end

endmodule