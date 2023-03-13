//Update the refcount of a node to 0, then try to use the capability
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

    la a1, val
    SCC(a3, a1)

    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a3, a2)
    QUERY(a2)

    CAPPRINT(a3)

    LWS(a5, a3)
    CAPPRINT(a5)

    li a4, -1
    RCUPDATE(a2, a4)

    LWS(a6, a3)
    CAPPRINT(a6)

    j exit

.global val
val:
    .word 42