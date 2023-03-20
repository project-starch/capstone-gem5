//store 64 bits and load 8 bits
#include "asm_insn.h"

.global _start
_start:
    //boilerplate
    INIT_RWX_CAP(a3)

    li a1, 0x40000
    li a2, 0x50000
    CAPBOUND(a3, a1, a2)
    //end boilerplate

    SCC(a3, a1)
    CAPPRINT(a3)

    li a1, 0xFFFFFF42
    SHC(a3, a1)

    LBS(a4, a3)
    CAPPRINT(a4)

    j exit