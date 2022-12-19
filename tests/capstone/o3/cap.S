#include "asm_insn.h"

.global _start
_start: 
    CAPCREATE(a3)
    li a1, 0
    CAPTYPE(a3, a1)
    li a2, 0xffff
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)

    j exit

