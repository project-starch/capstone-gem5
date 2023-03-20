//store 8 bits and load 64 bits
//This case (store small data and try to load bigger data)
//is currently failing
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
    SBC(a3, a1)

    LWS(a4, a3)
    CAPPRINT(a4)

    j exit