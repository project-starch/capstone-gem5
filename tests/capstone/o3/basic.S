#include "asm_insn.h"

.global _start
_start:
    li a1, 1
    lw a2,val
    add a2,a1,a2
    j exit
val:
    .word 2
