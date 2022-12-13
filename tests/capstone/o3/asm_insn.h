#define REVOKE(r)        .insn i 0x77, 0x1, x0, r, 0

#define CALL(rd,rs1)      .insn i 0x77, 0x2, rd, rs1, 0
#define JMP(r)           .insn i 0x77, 0x6, x0, r, 0
#define JNZ(rd,rs1)         .insn i 0x77, 0x7, rd, rs1, 0

#define SHRINK(rd,rs1,rs2)    .insn r 0x6B, 0x0, 0x0, rd, rs1, rs2
#define LT(rd,rs1,rs2)        .insn r 0x6B, 0x0, 0x1, rd, rs1, rs2
#define TIGHTEN(rd,rs1)     .insn i 0x6B, 0x1, rd, rs1, 0
#define DELIN(r)         .insn i 0x6B, 0x2, r, x0, 0
#define SCC(rd,rs1)         .insn i 0x6B, 0x3, rd, rs1, 0
#define INIT(r)          .insn i 0x6B, 0x4, r, x0, 0
#define SEAL(r)           .insn i 0x6B, 0x5, r, x0, 0
#define ADD2(rd,rs1)        .insn i 0x6B, 0x6, rd, rs1, 0

//Test instructions for node ops
#define QUERY(r)         .insn i 0x5F, 0x0, x0, r, 0
#define DROP(r)          .insn i 0x5F, 0x1, x0, r, 0
#define RCUPDATE(r)      .insn i 0x5F, 0x2, x0, r, 0
#define ALLOC(r)         .insn i 0x5F, 0x3, x0, r, 0
#define REVOKET(r)       .insn i 0x5F, 0x4, x0, r, 0