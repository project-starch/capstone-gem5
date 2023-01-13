.global _start
_start:
    li a0, 1
    la a2, data
    ld a1, 0(a2)
    add a1, a1, a0
    sd a1, 0(a2)
    j _start

    j exit

data:
.word 0
.word 0
