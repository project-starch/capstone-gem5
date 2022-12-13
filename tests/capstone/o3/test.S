#include "asm_insn.h"

.global _start
_start: 
    li a1, 33
    li a2, 0
    add a2, a1, a2
    REVOKE(a1)
    
    j exit

