#include "asm_insn.h"

.global _start
_start: 
    #REVOKE(a1)
    li a1, NODE_ID_INVALID
    ALLOC(a2,a1)
    QUERY(a2)
    DROP(a2)
    QUERY(a2)

    ALLOC(a2,a1)
    QUERY(a2)
    DROP(a2)
    QUERY(a2)
    
    j exit

