//Simple looping using jnz
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

    la a1, loop
    SCC(a3, a1)
    CAPPRINT(a3)

    li a2, 10

    loop:
    CAPPRINT(a2)
    addiw a2, a2, -1
    JNZ(a3, a2)
    j exit
