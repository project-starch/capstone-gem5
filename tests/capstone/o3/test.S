#include "asm_insn.h"

.global _start
_start: 
    #REVOKE(a1)
    li a1, NODE_ID_INVALID
    ALLOC(a1)
    li a1, 30
    #RCUPDATE(a1,a2)
    QUERY(a1)
    
    j exit

