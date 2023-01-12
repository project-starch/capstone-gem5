#ifndef __CAPSTONE_ASM_HH__
#define __CAPSTONE_ASM_HH__

static void print_tag(void* v) {
    asm(".insn r 0x5B, 0x0, 0x7, x0, %0, x0"
            : : "r"(v));
}

#endif
