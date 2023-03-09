#include "asm_insn.h"

.global _start
_start:
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

    la a5, val
    SCC(a3, a5)
    CAPPRINT(a3)

    CAPCREATE(a2)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a2, a1)
    
    li a1, 0
    li a6, 0x99999
    CAPBOUND(a2, a1, a6)
    CAPPRINT(a2)

    SDC(a3, a2)
    CAPPRINT(a2) # should be scrubbed
    
    fence
    li a4, 0
    LDC(a4, a3)
    CAPPRINT(a4)
    
    fence
    li a5, 0
    LDS(a5, a3)
    CAPPRINT(a5)
    
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