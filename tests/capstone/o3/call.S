#include "asm_insn.h"

.global _start
_start:
    # sealed capability for context
    CAPCREATE(a3)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a3, a1)

    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a3, a2)
    CAPPRINT(a3)


    li a1, CAP_PERM_RW
    CAPPERM(a3, a1)

    la a1, _context
    li a2, 0x100000
    CAPBOUND(a3, a1, a2)
    CAPPRINT(a3)
    
    # capability for new PC
    CAPCREATE(a4)
    li a1, CAP_TYPE_LIN
    CAPTYPE(a4, a1)
    
    li a1, NODE_ID_INVALID
    ALLOC(a2, a1)
    CAPNODE(a4, a2)
    CAPPRINT(a4)

    li a1, CAP_PERM_RX
    CAPPERM(a4, a1)
    la a1, _stub
    la a2, _stub_end
    CAPPRINT(a1) 
    CAPPRINT(a2)

    CAPBOUND(a4, a1, a2)
    CAPPRINT(a4)
    
    # write capability to the start of the context region
    SDC(a3, a4)
    SEAL(a3)
    
    .rept 200
    nop
    .endr

    li a2, 0
    CALL(a2, a3)
    
    j exit

_stub:
    li a1, 0xdeadbeef
    CAPPRINT(a1)
    
    EXIT_STUB

_stub_end:

.align REG_SIZE
_context:
    .fill 64, REG_SIZE, 0 # FIXME: should be 32