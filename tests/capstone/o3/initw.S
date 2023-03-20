//try to initialize an uninit cap after its cursor = end
#include "asm_insn.h"

.global _start
_start:
    //boilerplate
    INIT_RWX_CAP(a3)

    li a1, 0x40000
    li a2, 0x40008
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)
    //end boilerplate

    SCC(a3, a1)

    li a1, CAP_TYPE_UNINIT
    CAPTYPE(a3, a1)

    CAPPRINT(a3)

    #LWS(a4, a3) #should fail

    li a1, 0xFF42FF42FF42FF42
    SHC(a3, a1)

    CAPPRINT(a3)

    INIT(a3)

    li a2, 0x40000
    SCC(a3, a2)

    CAPPRINT(a3)

    LHS(a4, a3)
    CAPPRINT(a4)

    j exit