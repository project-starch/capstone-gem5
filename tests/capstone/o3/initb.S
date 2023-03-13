//try to initialize an uninit cap after its cursor = end
#include "asm_insn.h"

.global _start
_start:
    //boilerplate
    CAPCREATE(a3)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a3, a1)
    
    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a3, a2)

    li a1, CAP_PERM_RWX
    CAPPERM(a3, a1)

    li a1, 0x40000
    li a2, 0x40001
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)
    //end boilerplate

    SCC(a3, a1)

    li a1, CAP_TYPE_UNINIT
    CAPTYPE(a3, a1)

    CAPPRINT(a3)

    #LWS(a4, a3) #should fail

    li a1, 0x42
    SBC(a3, a1)

    CAPPRINT(a3)

    INIT(a3)

    li a2, 0x40000
    SCC(a3, a2)

    CAPPRINT(a3)

    LBS(a4, a3)
    CAPPRINT(a4)

    j exit