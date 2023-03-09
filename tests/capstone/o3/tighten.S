//delegate a capability with reduced perms/bounds
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

    MREV(a4,a3)
    CAPPRINT(a4)

    li a1, CAP_PERM_RO
    TIGHTEN(a3, a1)

    li a1, 0
    li a2, 0x500000
    SHRINK(a3, a1, a2)
    CAPPRINT(a3)

    REVOKE(a4)
    CAPPRINT(a4)

    j exit