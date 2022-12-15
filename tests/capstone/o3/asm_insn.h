#define REVOKE(reg)        .insn r 0x5B, 0x1, 0x0, x0, reg, x0

#define CALL(rd,rs1)      .insn r 0x5B, 0x1, 0x1, rd, rs1, x0
#define JMP(reg)           .insn r 0x5B, 0x1, 0x2, x0, reg, x0
#define JNZ(rd,rs1)         .insn r 0x5B, 0x1, 0x3, rd, rs1, x0

#define SHRINK(rd,rs1,rs2)    .insn r 0x5B, 0x1, 0x4, rd, rs1, rs2
#define LT(rd,rs1,rs2)        .insn r 0x5B, 0x1, 0x5, rd, rs1, rs2
#define TIGHTEN(rd,rs1)     .insn r 0x5B, 0x1, 0x6, rd, rs1, x0
#define DELIN(reg)         .insn r 0x5B, 0x1, 0x7, reg, x0, x0
#define SCC(rd,rs1)         .insn r 0x5B, 0x1, 0x8, rd, rs1, x0
#define INIT(reg)          .insn r 0x5B, 0x1, 0x9, reg, x0, x0
#define SEAL(reg)           .insn r 0x5B, 0x1, 0xa, reg, x0, x0
#define ADD2(rd,rs1)        .insn r 0x5B, 0x1, 0xb, rd, rs1, x0

// Test instructions for node ops
#define QUERY(reg)         .insn r 0x5B, 0x0, 0x0, x0, reg, x0
#define DROP(reg)          .insn r 0x5B, 0x0, 0x1, reg, reg, x0
#define RCUPDATE(rs1,rs2)      .insn r 0x5B, 0x0, 0x2, x0, rs1, rs2
#define ALLOC(reg)         .insn r 0x5B, 0x0, 0x3, reg, reg, x0
#define REVOKET(reg)       .insn r 0x5B, 0x0, 0x4, x0, reg, x0


#define NODE_ID_INVALID ((-1) & ((1 << 31) - 1))
