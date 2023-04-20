/*
 * Copyright 2023 National University of Singapore
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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




#include "arch/riscvcapstone/linux/syscall_emul.hh"
#include "debug/CapstoneAlloc.hh"
#include "arch/riscvcapstone/node_controller.hh"
#include "arch/riscvcapstone/typing.hh"

namespace gem5::RiscvcapstoneISA {


SyscallReturn
notifymallocFunc(SyscallDesc* desc, ThreadContext* tc,
        uint64_t addr, uint64_t size) {
    DPRINTF(CapstoneAlloc, "malloc: %llx, %llu\n", addr, size);

    BaseSimpleCPUWithNodeController* cpu = 
        dynamic_cast<BaseSimpleCPUWithNodeController*>(tc->getCpuPtr());
    if(cpu) {
        cpu->getNodeController()->
            allocObject(SimpleAddrRange((Addr)addr, (Addr)(addr + size)));
    } else {
        DPRINTF(CapstoneAlloc, "malloc: warning! cpu does not have a node controller!\n");
    }
    return addr;
}

SyscallReturn
notifyfreeFunc(SyscallDesc* desc, ThreadContext* tc,
        uint64_t addr) {
    DPRINTF(CapstoneAlloc, "free: %llx\n", addr);


    BaseSimpleCPUWithNodeController* cpu = 
        dynamic_cast<BaseSimpleCPUWithNodeController*>(tc->getCpuPtr());
    if(cpu) {
        cpu->getNodeController()->freeObject((Addr)addr);
    } else {
        DPRINTF(CapstoneAlloc, "free: warning! cpu does not have a node controller!\n");
    }
    return SyscallReturn();
}

// FIXME: the return value ovewrites registers. Needs to clean up the capabilities inside

}

