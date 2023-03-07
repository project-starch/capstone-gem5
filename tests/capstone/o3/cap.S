#include "asm_insn.h"

.global _start
_start: 
    CAPCREATE(a3)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a3, a1)

    li a1, CAP_PERM_RW
    CAPPERM(a3, a1)

    li a1, 0
    li a2, 0xffff
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)
    
    li a1, 0
    li a2, 0xefff
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)

    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a3, a2)
    CAPPRINT(a3)

    j exit

