//Stress tcache, seq access
#include "asm_insn.h"

.global _start
_start:

    INIT_RWX_CAP(a6)
    li a1, 0x30000
    li a2, 0x10000000
    CAPBOUND(a6, a1, a2)
    CAPPRINT(a6)

    /**
     * store some dummy value and check if it can
     * can be correctly loaded by the loaded capabilities
     */
    li a7, 0x40000
    SCC(a6, a7)
    li a1, 0x42
    SDS(a6, a1)

    li a3, xx
    li a5, 0
    li a7, 0x50000
    SCC(a6, a7)

    //-------make cap-------//
    CAPCREATE(a4)
    li a2, CAP_TYPE_NONLIN
    CAPTYPE(a4, a2)
    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a4, a2)
    li a1, 0x40000
    li a2, 0x50000
    CAPBOUND(a4, a1, a2)
    li a1, CAP_PERM_RO
    CAPPERM(a4, a1)
    //-------end make cap-------//

_loop:
    addi a3,a3,-1
    STC(a6, a4)
    addi a7, a7, 0x10
    SCC(a6, a7)
    bne a3, a5, _loop

//workaround for assert(flags.isSet(STATIC_DATA|DYNAMIC_DATA))
//for whatever reason that error is not coming anymore
//.rept 1000
//nop
//.endr

//-------warmup end-------//
li t3, 1
li t4, 0
M5_EXIT

_end_loop:
    addi t4, t4, 1
    li a3, xx
    CAPPRINT(a3)
    li a7, 0x50000
    SCC(a6, a7)
    li a4, 0x42

_loop2:
    addi a3,a3,-1
    LTC(a2, a6)      #causes 1 read + 1 write access to tcache for linear types
    addi a7, a7, 0x10
    LDS(a1, a2)
    bne a1, a4, _test_fail
    SCC(a6, a7)
    bne a3, a5, _loop2
    
    bne t4, t3, _end_loop
    j exit

_test_fail:
    li a0, 0
    li a7, 10
    ecall