#ifdef USE_UNCOMPRESSED
#define REG_SIZE  0x20
#else
#define REG_SIZE  0x10
#endif


#define REVOKE(reg)             .insn r 0x5B, 0x1, 0x0, x0, reg, x0

#define CALL(rd,rs1)            .insn r 0x5B, 0x1, 0x1, rd, rs1, x0
#define JMP(reg)                .insn r 0x5B, 0x1, 0x2, x0, reg, x0
#define JNZ(rd,rs1)             .insn r 0x5B, 0x1, 0x3, rd, rs1, x0

#define SHRINK(rd,rs1,rs2)      .insn r 0x5B, 0x1, 0x4, rd, rs1, rs2
#define LT(rd,rs1,rs2)          .insn r 0x5B, 0x1, 0x5, rd, rs1, rs2
#define TIGHTEN(rd,rs1)         .insn r 0x5B, 0x1, 0x6, rd, rs1, x0
#define DELIN(reg)              .insn r 0x5B, 0x1, 0x7, reg, x0, x0
#define SCC(rd,rs1)             .insn r 0x5B, 0x1, 0x8, rd, rs1, x0
#define INIT(reg)               .insn r 0x5B, 0x1, 0x9, reg, x0, x0
#define SEAL(reg)               .insn r 0x5B, 0x1, 0xa, reg, x0, x0
#define ADD2(rd,rs1)            .insn r 0x5B, 0x1, 0xb, rd, rs1, x0
#define MOVC(rd,rs1)            .insn r 0x5B, 0x1, 0xc, rd, rs1, x0
#define LDC(rd,rs1)             .insn r 0x5B, 0x1, 0xe, rd, rs1, x0
#define LCC(rd,rs1)             .insn r 0x5B, 0x1, 0xd, rd, rs1, x0
#define SDC(rd,rs1)             .insn r 0x5B, 0x1, 0xf, rd, rs1, x0
#define DROPI(reg)              .insn r 0x5B, 0x1, 0x10, x0, reg, x0
#define MREV(rd,rs1)            .insn r 0x5B, 0x1, 0x11, rd, rs1, x0
#define SPLIT(rd,rs1,rs2)       .insn r 0x5B, 0x1, 0x12, rd, rs1, rs2
#define LDS(rd,rs1)             .insn r 0x5B, 0x1, 0x13, rd, rs1, x0

// Test instructions for node ops
#define QUERY(reg)              .insn r 0x5B, 0x0, 0x0, x0, reg, x0
#define DROP(reg)               .insn r 0x5B, 0x0, 0x1, x0, reg, x0
#define RCUPDATE(rs1,rs2)       .insn r 0x5B, 0x0, 0x2, x0, rs1, rs2
#define ALLOC(rd,rs1)           .insn r 0x5B, 0x0, 0x3, rd, rs1, x0
#define REVOKET(reg)            .insn r 0x5B, 0x0, 0x4, x0, reg, x0

// Test instructions for capability ops
#define CAPCREATE(reg)          .insn r 0x5B, 0x0, 0x5, reg, x0, x0
#define CAPTYPE(rd,rs1)         .insn r 0x5B, 0x0, 0x6, rd, rs1, x0
#define CAPNODE(rd,rs1)         .insn r 0x5B, 0x0, 0x7, rd, rs1, x0
#define CAPPERM(rd,rs1)         .insn r 0x5B, 0x0, 0x8, rd, rs1, x0
#define CAPBOUND(rd,rs1,rs2)    .insn r 0x5B, 0x0, 0x9, rd, rs1, rs2
#define CAPPRINT(rs1)           .insn r 0x5B, 0x0, 0xa, x0, rs1, x0

// Test instructions for tag manipulation
#define TAGSET(rs1,rs2)         .insn r 0x5B, 0x0, 0xb, x0, rs1, rs2
#define TAGGET(rs1,rs2)         .insn r 0x5B, 0x0, 0xc, x0, rs1, rs2
#define SD(rs1,rs2)             .insn r 0x43, 0x2, 0x0, x0, rs1, rs2

#define NODE_ID_INVALID ((-1) & ((1 << 31) - 1))

// Capability-related constants
#define CAP_PERM_NA 0
#define CAP_PERM_RO 1
#define CAP_PERM_RX 2
#define CAP_PERM_RW 3
#define CAP_PERM_RWX 4


#define CAP_TYPE_LIN 0
#define CAP_TYPE_NONLIN 1
#define CAP_TYPE_REV 2
#define CAP_TYPE_UNINIT 3
#define CAP_TYPE_SEALED 4
#define CAP_TYPE_SEALEDRET 5


