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

    la a1, _pprint
    SCC(a3, a1)
    CAPPRINT(a3)

    JMP(a3)

_resume:
    j exit


.align 16
_pprint:
    li a2, 0x22
    CAPPRINT(a2)

    j _resume