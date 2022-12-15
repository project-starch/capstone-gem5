#include "asm_insn.h"

.global _start
_start: 
    li a1, NODE_ID_INVALID
    ALLOC(a2,a1)
    QUERY(a2)
    li a3,1
    RCUPDATE(a2,a3)
    QUERY(a2)
    RCUPDATE(a2,a3)
    QUERY(a2)

    j exit

