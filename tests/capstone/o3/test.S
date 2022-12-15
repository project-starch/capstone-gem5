#include "asm_insn.h"

.global _start
_start: 
    #REVOKE(a1)
    li a1, NODE_ID_INVALID
    ALLOC(a2,a1)
    ALLOC(a3,a2)
    QUERY(a3)
    QUERY(a2)
    ALLOC(a4,a2)
    QUERY(a4)
    QUERY(a3)
    QUERY(a2)
    REVOKET(a2)
    QUERY(a4)
    QUERY(a3)
    QUERY(a2)

    REVOKET(a2)
    QUERY(a4)
    QUERY(a3)
    QUERY(a2)

    DROP(a2)
    QUERY(a4)
    QUERY(a3)
    QUERY(a2)

    j exit

