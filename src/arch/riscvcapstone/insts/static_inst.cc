/*
 * Copyright (c) 2015 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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

#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/insts/static_inst.hh"

#include "arch/riscvcapstone/pcstate.hh"
#include "arch/riscvcapstone/types.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvcapstoneISA
{

void
RiscvMicroInst::advancePC(PCStateBase &pcState) const
{
    auto &rpc = pcState.as<PCState>();
    if (flags[IsLastMicroop]) {
        rpc.uEnd();
    } else {
        rpc.uAdvance();
    }
}

void
RiscvMicroInst::advancePC(ThreadContext *tc) const
{
    PCState pc = tc->pcState().as<PCState>();
    if (flags[IsLastMicroop]) {
        pc.uEnd();
    } else {
        pc.uAdvance();
    }
    tc->pcState(pc);
}

Fault
RiscvStaticInst::completeAcc(ExecContext *xc, Trace::InstRecord *traceData) const {
    auto dyn_inst = dynamic_cast<o3::DynInst*>(xc);
    assert(dyn_inst);

    if(dyn_inst->getMemReadN() > 0) {
        return completeAcc(dyn_inst->getMemReadRes(0), xc, traceData);
    }

    return completeAcc(nullptr, xc, traceData);
}

} // namespace RiscvcapstoneISA
} // namespace gem5
