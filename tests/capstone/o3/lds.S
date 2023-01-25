#include "asm_insn.h"



.global _start
_start: 
    CAPCREATE(a3)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a3, a1)

    #li a1, CAP_PERM_RW
    li a1, 0
    CAPPERM(a3, a1)

    li a1, 0
    li a2, 0xfffffff
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)

    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a3, a2)
    CAPPRINT(a3)

    la a5, val
    SCC(a3, a5)

    CAPPRINT(a3)

    LDS(a4, a3)
    
    CAPPRINT(a4)

#    li a6, 63
#    SDC(a3, a6)
#
#    LDC(a4, a3)
#    CAPPRINT(a4)

    j exit

.global val
val:
    .word 42
