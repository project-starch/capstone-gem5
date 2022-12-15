#include "asm_insn.h"

.global _start
_start: 
    #REVOKE(a1)
    li a1, NODE_ID_INVALID
    ALLOC(a1)
    li a1, 1
    #REVOKE(a1)
    QUERY(a1)
    #RCUPDATE(a1,a2)
    
    j exit

