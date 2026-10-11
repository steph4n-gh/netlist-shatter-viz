module RiscVCore(
    input clk,
    input rst,
    input [31:0] instr,
    input [31:0] rdata,
    output [31:0] pc,
    output [31:0] alu_out,
    output [31:0] wdata,
    output mem_write
);
    // Program Counter
    reg [31:0] pc_reg;
    always @(posedge clk or posedge rst) begin
        if (rst) pc_reg <= 0;
        else pc_reg <= pc_reg + 4;
    end
    assign pc = pc_reg;

    // Decode Opcode fields
    wire [6:0] opcode = instr[6:0];
    wire [4:0] rd     = instr[11:7];
    wire [2:0] funct3 = instr[14:12];
    wire [4:0] rs1    = instr[19:15];
    wire [4:0] rs2    = instr[24:20];
    wire [6:0] funct7 = instr[31:25];

    // Immediate Decoder
    reg [31:0] imm;
    always @(*) begin
        case (opcode)
            7'b0010011: imm = {{20{instr[31]}}, instr[31:20]}; // I-type (addi, etc.)
            7'b0100011: imm = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type (sw, etc.)
            7'b1100011: imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type (beq, etc.)
            default:    imm = 32'b0;
        endcase
    end

    // Compact Register File: 8 registers x 16-bit to keep visualizer extremely responsive
    reg [15:0] regfile [0:7];
    wire [15:0] rf_rs1 = (rs1 < 8) ? regfile[rs1] : 16'b0;
    wire [15:0] rf_rs2 = (rs2 < 8) ? regfile[rs2] : 16'b0;
    
    always @(posedge clk) begin
        if (opcode == 7'b0110011 || opcode == 7'b0010011) begin // R-type or I-type writeback
            if (rd > 0 && rd < 8) begin
                regfile[rd] <= alu_out[15:0];
            end
        end
    end

    // ALU Control Decoders
    reg [3:0] alu_control;
    always @(*) begin
        case (opcode)
            7'b0110011: begin // R-type
                case (funct3)
                    3'b000: alu_control = (funct7[5]) ? 4'b0001 : 4'b0000; // Sub or Add
                    3'b111: alu_control = 4'b0010; // AND
                    3'b110: alu_control = 4'b0011; // OR
                    3'b100: alu_control = 4'b0100; // XOR
                    default: alu_control = 4'b0000;
                endcase
            end
            7'b0010011: begin // I-type
                case (funct3)
                    3'b000: alu_control = 4'b0000; // Addi
                    3'b111: alu_control = 4'b0010; // Andi
                    3'b110: alu_control = 4'b0011; // Ori
                    3'b100: alu_control = 4'b0100; // Xori
                    default: alu_control = 4'b0000;
                endcase
            end
            default: alu_control = 4'b0000;
        endcase
    end

    // ALU Execution Unit
    reg [31:0] alu_result;
    always @(*) begin
        case (alu_control)
            4'b0000: alu_result = {16'b0, rf_rs1} + ((opcode == 7'b0010011) ? imm : {16'b0, rf_rs2}); // Add / Addi
            4'b0001: alu_result = {16'b0, rf_rs1} - {16'b0, rf_rs2}; // Sub
            4'b0010: alu_result = {16'b0, rf_rs1} & ((opcode == 7'b0010011) ? imm : {16'b0, rf_rs2}); // And / Andi
            4'b0011: alu_result = {16'b0, rf_rs1} | ((opcode == 7'b0010011) ? imm : {16'b0, rf_rs2}); // Or / Ori
            4'b0100: alu_result = {16'b0, rf_rs1} ^ ((opcode == 7'b0010011) ? imm : {16'b0, rf_rs2}); // Xor / Xori
            default: alu_result = 32'b0;
        endcase
    end

    assign alu_out = alu_result;
    assign wdata = {16'b0, rf_rs2};
    assign mem_write = (opcode == 7'b0100011); // sw (Store Word)
endmodule
