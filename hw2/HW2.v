module ALU(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: and, 3: avg
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter AND  = 3'd3;
    parameter AVG  = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;
    reg         ready_;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = ready_;
    assign out = shreg;
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) 
                    case (mode)
                        0 : state_nxt = MUL;
                        1 : state_nxt = DIV;
                        2 : state_nxt = AND;
                        3 : state_nxt = AVG;
                    endcase
                else state_nxt = IDLE;
            end
            MUL : state_nxt = (counter != 31) ? MUL : OUT;
            DIV : state_nxt = (counter != 31) ? DIV : OUT;
            AND : state_nxt = OUT;
            AVG : state_nxt = OUT;
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    
    // Todo 2: Counter
    always @(*) begin
        if ((state == MUL) || (state == DIV)) counter_nxt = counter + 1; 
        else                                  counter_nxt = 0;
    end

    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case (state)
            MUL : begin
                if (shreg[0]) alu_out = shreg[63:32] + alu_in;
                else          alu_out = shreg[63:32];
            end
            DIV : alu_out = shreg[63:32] - alu_in;
            AND : alu_out = shreg[31:0] & alu_in;
            AVG : alu_out = (shreg[31:0] + alu_in) >> 1;
            default alu_out = 0;
        endcase
    end
    
    // Todo 4: Shift register
    always @(*) begin
        case (state)
            IDLE: begin
                if (valid) begin
                    shreg_nxt = {32'b0, in_A};
                    if (state_nxt == DIV) shreg_nxt = shreg_nxt << 1;                 
                end
                else shreg_nxt = 0;
            end
            MUL : shreg_nxt = {alu_out, shreg[31:1]};
            DIV : begin
                if (shreg[63:32] < alu_in) shreg_nxt = shreg << 1;
                else                      shreg_nxt = {alu_out[30:0], shreg[31:0], 1'b1};
                if (counter == 31) shreg_nxt = {1'b0, shreg_nxt[63:33], shreg_nxt[31:0]};
            end
            AND : shreg_nxt = {31'b0, alu_out};
            AVG : shreg_nxt = {31'b0, alu_out};
            OUT : shreg_nxt = 0;
            default : shreg_nxt = 0;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            counter <= 0;
            shreg   <= 0;
            alu_in  <= 0;
            ready_  <= 0;
        end
        else begin
            state   <= state_nxt;
            counter <= counter_nxt;
            shreg   <= shreg_nxt;
            alu_in  <= alu_in_nxt;
            ready_  <= (state_nxt == OUT) ? 1 : 0;
        end
    end

endmodule