.global _start
_start: 
    li a1, 33
    li a2, 0
    add a2, a1, a2
    .insn i 0x77, 0x01, a2, a1, 0
    
    j exit

