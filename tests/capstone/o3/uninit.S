//Increment cursor on store using an uninit capability.
//Show a load failure on using an uninit cap
#include "asm_insn.h"

.global _start
_start:
    //boilerplate
    CAPCREATE(a3)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a3, a1)

    li a1, CAP_PERM_RW
    CAPPERM(a3, a1)

    li a1, 0
    li a2, 0xfffffff
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)

    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a3, a2)
    CAPPRINT(a3)
    //end boilerplate

    la a5, val
    SCC(a3, a5)
    CAPPRINT(a3)

    li a1, CAP_TYPE_UNINIT
    CAPTYPE(a3, a1)

    CAPCREATE(a2)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a2, a1)
    CAPPRINT(a2)
    SWC(a3, a2)
    CAPPRINT(a2) # should be scrubbed
    CAPPRINT(a3)

    li a4, 0
    LWS(a4, a3) # this load should fail

    j exit

.global val
.align 16
val:
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0 