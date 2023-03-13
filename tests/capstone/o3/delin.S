//delin a cap and store a copy in the memory
//retrieve and try to use it
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

    li a1, 0
    li a2, 0x1000000ULL
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)
    //end boilerplate

    //nonlin cap to be copied
    CAPCREATE(a2)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a2, a1)
    
    li a1, NODE_ID_INVALID
    ALLOC(a4, a1)
    CAPNODE(a2, a4)

    li a1, CAP_PERM_RWX
    CAPPERM(a2, a1)

    li a1, 0
    li a4, 0x99999
    CAPBOUND(a2, a1, a4)
    CAPPRINT(a2)

    DELIN(a2)
    CAPPRINT(a2)

    la a1, val
    SCC(a3, a1)
    CAPPRINT(a3)

    SWC(a3, a2)
    CAPPRINT(a2)

    LWC(a4, a3)
    CAPPRINT(a4)
    CAPPRINT(a2)

    j exit

.align 16
.global val
val:
    .word 42