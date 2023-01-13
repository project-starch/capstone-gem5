/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "capstone_asm.h"

void* do_whatever(void* f) {
    return f;
}

void good(void* f){
    printf("%p\n", f);
}

int main(int argc, char* argv[])
{
    printf("Hello gem5!\n");
    fflush(stdout);
    void* s = malloc(15);
    PRINT_TAG(s);
    printf("Print tag below\n");
    PRINT_TAG(s);
    fflush(stdout);
    PRINT_TAG(s);
    void* c = (void*)((uintptr_t)do_whatever(s) + 2);
    PRINT_TAG(s);
    good(c);
    free(s);
    s = malloc(4);
    *(volatile char*)s = 0;
    PRINT_TAG(s);
    free(s);
    /*asm volatile ("li a0, 10\n\t"*/
            /*"call malloc\n\t"*/
            /*"call free\n\t");*/
    /*asm volatile("li a7, 3000\n\t"*/
            /*"li a0, 14\n\t"*/
            /*"li a1, 20\n\t"*/
            /*"ecall\n\t"*/
            /*"li a7, 3001\n\t"*/
            /*"ecall\n\t");*/
    /*void* s = malloc(4);*/
    /*free(s);*/
    return 0;
}

