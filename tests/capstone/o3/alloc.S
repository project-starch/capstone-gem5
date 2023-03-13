//Allocate to the rev tree in an infinite loop
#include "asm_insn.h"

.global _start
_start:
    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPPRINT(a2)
    j _start

    j exit
