//try to store and load various diff sizes to memory and initialize the cap
#include "asm_insn.h"

.global _start
_start:
    //boilerplate
    INIT_RWX_CAP(a3)

    li a1, 0x40000
    li a2, 0x40007
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)
    //end boilerplate

    SCC(a3, a1)

    li a1, CAP_TYPE_UNINIT
    CAPTYPE(a3, a1)

    CAPPRINT(a3)

    li a1, 0x42
    SBC(a3, a1)

    CAPPRINT(a3)

    li a1, 0xFF42
    S16C(a3, a1)

    CAPPRINT(a3)

    li a1, 0xFFFFFF42
    S32C(a3, a1)

    CAPPRINT(a3)

    INIT(a3)

    li a2, 0x40000
    SCC(a3, a2)

    CAPPRINT(a3)

    L32S(a4, a3)
    CAPPRINT(a4)

    li a2, 0x40001
    SCC(a3, a2)

    L16S(a4, a3)
    CAPPRINT(a4)

    j exit