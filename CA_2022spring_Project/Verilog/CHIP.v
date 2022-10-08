// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;

    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg    [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    reg    [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    // 5 stage (same as pipelined)
    reg [3:0] STATE, STATE_next;
    parameter INIT = 4'd0;
    // PC should have changed to correct PC when STATE change to IF
    parameter IF   = 4'd1;
    parameter ID   = 4'd2;
    parameter EX   = 4'd3;
    parameter MEM  = 4'd4;
    parameter WB   = 4'd5;
    // instruction after IF
    wire [31:0] instruction;
    // control signals
    wire       Branch, MemRead, MemtoReg, MemWrite, ALUSrc, regWrite_ctl;
    wire [3:0] ALUcontrol;
    // immediate calculated by imm_gen
    wire [31:0] immediate;
    // ALU signals
    wire [31:0] ALU_in_A, ALU_in_B, ALU_out, ALU_result, mulDiv_result;
    wire        mulDiv_ready;
    reg         mulDiv_valid, EX_ready;

    wire [6:0] opcode;
    assign mem_addr_I = PC;
    assign instruction = mem_rdata_I;
    assign opcode = mem_rdata_I[6:0];
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign rd =  mem_rdata_I[11:7];

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: any combinational/sequential circuit
    control ctl(
        .instruction(instruction),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOp(),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(regWrite_ctl),
        .ALUcontrol(ALUcontrol));
    imm_gen ig(
        .instruction(instruction),
        .immediate(immediate));

    // mux: ALU input
    assign ALU_in_A = (opcode == 7'b0010111) ? PC : rs1_data;
    assign ALU_in_B = (ALUSrc) ? immediate : rs2_data;

    // mux: ALU output
    ALU alu(
        .mode(ALUcontrol),
        .ALU_in_A(ALU_in_A),
        .ALU_in_B(ALU_in_B),
        .ALU_result(ALU_result));

    mulDiv muxalu(
        .clk(clk),
        .rst_n(rst_n),
        .valid(mulDiv_valid),
        .ready(mulDiv_ready),
        .mode(ALUcontrol),
        .ALU_in_A(ALU_in_A),
        .ALU_in_B(ALU_in_B),
        .mulDiv_result(mulDiv_result));

    always @(*) begin
        case (STATE)
            EX: begin
                mulDiv_valid = (ALUcontrol == 4'b1100) ? 1 : 0;
                EX_ready = (mulDiv_ready == 1 || ALUcontrol != 4'b1100) ? 1 : 0;
            end
            default: begin
                mulDiv_valid = 0;
                EX_ready = 1;
            end
        endcase
    end

    assign ALU_out = (ALUcontrol == 4'b1100) ? mulDiv_result : ALU_result;

    // state combinational
    always @(*) begin
        case (STATE)
            INIT:    STATE_next = IF;
            IF:      STATE_next = ID;
            ID:      STATE_next = EX;
            EX:      STATE_next = (EX_ready || ALUcontrol != 4'b1100) ? MEM : EX;
            MEM:     STATE_next = WB;
            WB:      STATE_next = IF;
            default: STATE_next = INIT;
        endcase
    end

    // PC combinational
    always @(*) begin
        case (STATE)
            WB: case (opcode)
                    7'b1101111: // jal
                        PC_nxt = PC + immediate;
                    7'b1100111: // jalr
                        PC_nxt = ALU_out;
                    7'b1100011: // branch
                        PC_nxt = (ALU_out == 0) ? PC + immediate : PC + 4;
                    default:
                        PC_nxt = PC + 4;
                endcase
            default:
                PC_nxt = PC;
        endcase
    end

    // Data memory
    wire [31:0] DM_rdata; // data read from Data memory
    // connect Data memory
    assign mem_addr_D = ALU_out;// #TOBECONNECT(ALUOUT);
    assign mem_wdata_D = rs2_data; // #TODO don't need to from alu?
    assign mem_wen_D = (MemWrite) ? 1 : 0;
    assign DM_rdata = mem_rdata_D;
    assign regWrite = (STATE == WB) ? regWrite_ctl : 0;

    // reg_file input combinational
    always @(*) begin
        if(opcode == 7'b1101111) // jal
            rd_data = PC + 4;
        else if(opcode == 7'b1100111) // jalr
            rd_data = PC + 4;
        else if(opcode == 7'b0010111) // auipc
            rd_data = PC + immediate;
        else if(MemtoReg)
            rd_data = mem_rdata_D;
        else
            rd_data = ALU_out;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            STATE <= INIT;
        end
        else begin
            PC <= PC_nxt;
            STATE <= STATE_next;
        end
    end
endmodule

module control(instruction, Branch, MemRead, MemtoReg, ALUOp,
        MemWrite, ALUSrc, RegWrite, ALUcontrol);
    input [31:0] instruction;
    output       Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    output [1:0] ALUOp; // not used
    output reg [3:0] ALUcontrol;

    /*
    Branch
        0: PC_next = PC + 4
        1: PC_next = PC + proccessed imm
    MemRead
        1: read data from data memory
    MemtoReg
        0: data from alu output
        1: data from data memory output
    ALUOp
        below
    MemWrite
        1: write data to data memory
    ALUSrc
        0: data2 from register
        1: data2 from imm gen output
    RegWrite
        1: write data to register
    ALUcontrol
        below

    /*         funct7, fuct3, opcode, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, ALUcontrol
    R-type
        add  : 0000000, 000, 0110011, 0,      0,       0,        10,    0,        0,      1,        0
        sub  : 0100000, 000, 0110011, 0,      0,       0,        10,    0,        0,      1,        0
        xor  : 0000000, 100, 0110011, 0,      0,       0,        10,    0,        0,      1,        0
        mul  : 0000001, 000, 0110011, 0,      0,       0,        10,    0,        0,      1,        0
    I-type
        lw   :          010, 0000011, 0,      1,       1,        00,    0,        1,      1,        0
        addi :          000, 0010011, 0,      0,       0,        00,    0,        1,      1,        0
        slti :          010, 0010011, 0,      0,       0,        00,    0,        1,      1,        0
        slli : 0000000, 001, 0010011, 0,      0,       0,        00,    0,        1,      1,        0
        srli : 0000000, 101, 0010011, 0,      0,       0,        00,    0,        1,      1,        0
    S-type
        sw   :          010, 0100011, 0,      0,       0,        00,    1,        1,      0,        0
    SB-type
        beq  :          000, 1100011, 1,      0,       0,        01,    0,        0,      0,        0
        bge  :          101, 1100011, 1,      0,       0,        01,    0,        0,      0,        0
    U-type
        auipc:               0010111, 0,      0,       0,        00,    0,        1,      0,        0
    UJ-type
        jal  :               1101111, 1,      0,       0,        00,    0,        0,      1,        0
        jalr :               1100111, 1,      0,       0,        00,    0,        0,      1,        0
    */

    wire [6:0] opcode;
    wire [6:0] funct7;
    wire [2:0] funct3;

    assign opcode = instruction[6:0];
    assign funct7 = instruction[31:25];
    assign funct3 = instruction[14:12];

    assign Branch =   (opcode == 7'b1100011 || opcode == 7'b1101111 ||
                       opcode == 7'b1100111) ? 1 : 0;
    assign MemRead =  (opcode == 7'b0000011) ? 1 : 0;
    assign MemtoReg = (opcode == 7'b0000011) ? 1 : 0;
    assign MemWrite = (opcode == 7'b0100011) ? 1 : 0;
    assign ALUSrc =   (opcode == 7'b0000011 || opcode == 7'b0010011 ||
                       opcode == 7'b0100011 || opcode == 7'b0010111 ||
                       opcode == 7'b1101111 || opcode == 7'b1100111) ? 1 : 0;
    assign RegWrite = (opcode == 7'b0110011 || opcode == 7'b0000011 ||
                       opcode == 7'b0010011 || opcode == 7'b0010111 ||
                       opcode == 7'b1101111 || opcode == 7'b1100111) ? 1 : 0;

    // ALUcontrol
    // 0000 AND:
    // 0001 OR:
    // 0010 add: add, auipc, jal, jalr, lw, sw, addi,
    // 0011 xor: xor
    // 0110 subtract: beq, slti, sub
    // 0111 set less than: slti, bge
    // 1000 shift left: slli
    // 1001 shift right: srli
    // 1100 multiply: mul
    always @(*) begin
        case (opcode)
            7'b0110011:// R-type
                case ({funct7, funct3})
                    10'b0000000_000: // add
                        ALUcontrol = 4'b0010;
                    10'b0100000_000: // sub
                        ALUcontrol = 4'b0110;
                    10'b0000000_100: // xor
                        ALUcontrol = 4'b0011;
                    10'b0000001_000: // mul
                        ALUcontrol = 4'b1100;
                    default:
                        ALUcontrol = 4'b0;
                endcase
            7'b0000011: // I-type, lw
                ALUcontrol = 4'b0010;
            7'b0010011:
                case (funct3)
                    3'b000: // I-type, addi
                        ALUcontrol = 4'b0010;
                    3'b010: // I-type, slti
                        ALUcontrol = 4'b0111;
                    3'b001: // I-type, slli
                        ALUcontrol = 4'b1000;
                    3'b101: // I-type, srli
                        ALUcontrol = 4'b1001;
                    default:
                        ALUcontrol = 4'b0;
                endcase
            7'b0100011: // S-type, sw
                ALUcontrol = 4'b0010;
            7'b1100011: 
                case (funct3)
                    3'b000: // B-type, beq
                        ALUcontrol = 4'b0110;
                    3'b101: // B-type, bge
                        ALUcontrol = 4'b0111;
                    default: 
                        ALUcontrol = 4'b0;
                endcase
            7'b0010111: // U-type, auipc
                ALUcontrol = 4'b0010;
            7'b1101111: // J-type, jal
                ALUcontrol = 4'b0;
            7'b1100111: // J-type, jalr
                ALUcontrol = 4'b0010;
            default:
                ALUcontrol = 4'b0;
        endcase
    end

endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module imm_gen(instruction, immediate);
    input [31:0] instruction;
    output reg [31:0] immediate;

    wire [6:0] opcode;
    assign opcode = instruction[6:0];

    always @(*) begin
        case (instruction[6:0])
            7'b0000011: // I-type, lw
                immediate = {{20{instruction[31]}},instruction[31:20]};
            7'b0010011: // I-type, addi/slti
                immediate = {{20{instruction[31]}},instruction[31:20]};
            7'b0100011: // S-type, sw
                immediate = {{20{instruction[31]}},instruction[31:25],instruction[11:7]};
            7'b1100011: // B-type, beq
                immediate = {{19{instruction[31]}},instruction[31],instruction[7],instruction[30:25],instruction[11:8],1'b0};
            7'b0010111: // U-type, auipc
                immediate = {instruction[31:12],12'b0};
            7'b1101111: // J-type, jal
                immediate = {{11{instruction[31]}},instruction[31],instruction[19:12],instruction[20],instruction[30:21],1'b0};
            7'b1100111: // I-type, jalr
                immediate = {{20{instruction[31]}},instruction[31:20]};
            default:
                immediate = 32'b0;
        endcase
    end

endmodule

module ALU(mode, ALU_in_A, ALU_in_B, ALU_result);
    input [3:0] mode;
    input [31:0] ALU_in_A, ALU_in_B;
    output reg [31:0] ALU_result;

    always @(*) begin
        case(mode)
            4'b0000: // AND
                ALU_result = ALU_in_A & ALU_in_B;
            4'b0001: // OR
                ALU_result = ALU_in_A | ALU_in_B;
            4'b0010: // add: add, auipc, jalr, lw, sw, addi,
                ALU_result = ALU_in_A + ALU_in_B;
            4'b0011: // xor: xor
                ALU_result = ALU_in_A ^ ALU_in_B;
            4'b0110: // subtract: beq, sub
                ALU_result = ALU_in_A - ALU_in_B;
            4'b0111: // set less than: slti, bge
                ALU_result = ALU_in_A < ALU_in_B;
            4'b1000: // shift left: slli
                ALU_result = ALU_in_A << ALU_in_B;
            4'b1001: // shift right: srli
                ALU_result = ALU_in_A >> ALU_in_B;
            default:
                ALU_result = 32'b0;
        endcase
    end

endmodule

module mulDiv(clk, rst_n, valid, ready, mode, ALU_in_A, ALU_in_B, mulDiv_result);
    input         clk, rst_n;
    input         valid;
    input  [3:0]  mode;
    output        ready;
    input  [31:0] ALU_in_A, ALU_in_B;
    output [63:0] mulDiv_result;

    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter OUT  = 3'd2;

    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    assign ready = (state_nxt == OUT) ? 1 : 0;
    assign mulDiv_result = shreg;

    always @(*) begin
        case(state)
            IDLE: begin
                state_nxt = (valid) ? MUL : IDLE;
                counter_nxt = 0;
                alu_in_nxt = (valid) ? ALU_in_B : 0;
                alu_out = 0;
                shreg_nxt = (valid) ? {32'b0, ALU_in_A} : 0;
            end
            MUL : begin
                state_nxt = (counter < 5'd31) ? MUL : OUT;
                counter_nxt = counter + 1;
                alu_in_nxt = alu_in;
                alu_out = (shreg[0]) ? shreg[63:32] + alu_in : shreg[63:32];
                shreg_nxt = {alu_out, shreg[31:1]};
            end
            OUT : begin
                state_nxt = IDLE;
                counter_nxt = counter;
                alu_in_nxt = 0;
                alu_out = 0;
                shreg_nxt = shreg;
            end
            default : begin
                state_nxt = IDLE;
                counter_nxt = 0;
                alu_in_nxt = alu_in;
                alu_out = 0;
                shreg_nxt = shreg;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            counter <= 0;
            shreg   <= 0;
            alu_in  <= 0;
        end
        else begin
            state   <= state_nxt;
            counter <= counter_nxt;
            shreg   <= shreg_nxt;
            alu_in  <= alu_in_nxt;
        end
    end

endmodule