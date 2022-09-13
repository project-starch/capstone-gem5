/*
 * Copyright (c) 2015 RISC-V Foundation
 * Copyright (c) 2017 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
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

#include "arch/riscvcapstone/insts/standard.hh"

#include <sstream>
#include <string>

#include "arch/riscvcapstone/insts/static_inst.hh"
#include "arch/riscvcapstone/regs/misc.hh"
#include "arch/riscvcapstone/utility.hh"
#include "arch/riscvcapstone/node_controller.hh"
#include "arch/riscvcapstone/ncache_cpu.hh"
#include "cpu/exec_context.hh"
#include "cpu/simple/exec_context.hh"
#include "cpu/static_inst.hh"
#include "debug/CapstoneNodeOps.hh"

namespace gem5
{

namespace RiscvcapstoneISA
{

std::string
RegOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(0));
    if (_numSrcRegs >= 2)
        ss << ", " << registerName(srcRegIdx(1));
    if (_numSrcRegs >= 3)
        ss << ", " << registerName(srcRegIdx(2));
    return ss.str();
}

std::string
CSROp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    auto data = CSRData.find(csr);
    if (data != CSRData.end())
        ss << data->second.name;
    else
        ss << "?? (" << std::hex << "0x" << csr << std::dec << ")";
    if (_numSrcRegs > 0)
        ss << ", " << registerName(srcRegIdx(0));
    else
        ss << uimm;
    return ss.str();
}

std::string
SystemOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    if (strcmp(mnemonic, "fence_vma") == 0) {
        std::stringstream ss;
        ss << mnemonic << ' ' << registerName(srcRegIdx(0)) << ", " <<
            registerName(srcRegIdx(1));
        return ss.str();
    }

    return mnemonic;
}

std::string
EcallOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    return mnemonic;
}

InstStateMachinePtr
EcallOp::getStateMachine(ExecContext* xc) const {
    RegVal num = xc->tcBase()->readIntReg(RiscvcapstoneISA::SyscallNumReg);
    switch(num) {
        case 3000: // malloc
            return std::make_shared<MallocStateMachine>();
        case 3001:
            return std::make_shared<FreeStateMachine>();
        default:
            return std::make_shared<DummyInstStateMachine>();

    }
}

// Do three things
// 1. Register the range of the object
// 2. Allocate a new revocation node
// 3. Record that the register contains a capability with the specific node ID
void
MallocStateMachine::setup(ExecContext* xc) {
    //SimpleExecContext* sxc = dynamic_cast<SimpleExecContext*>(xc);
    //panic_if(sxc == NULL, "non-SimpleExecContext unVksupported.");
    
    NodeControllerAllocate* cmd = new NodeControllerAllocate();
    cmd->parentId = NODE_ID_INVALID; 

    TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "non ncache-cpu unsupported.");
    cpu->sendNCacheCommand(cmd);

    state = MALLOC_ALLOC_NODE;
}

bool
MallocStateMachine::finished(ExecContext* xc) const {
    return state == MALLOC_DONE;
}

Fault
MallocStateMachine::transit(ExecContext* xc, PacketPtr pkt) {
    NodeID node_id = pkt->getRaw<NodeID>();
    DPRINTF(CapstoneNodeOps, "Allocated node %u\n", node_id);

    TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "non ncache-cpu unsupported.");

    cpu->node_controller->addCapTrack(CapLoc::makeReg(xc->tcBase()->threadId(), SyscallNumReg), 
            node_id);
    
    state = MALLOC_DONE;
    return NoFault;
}

void
FreeStateMachine::setup(ExecContext* xc) {
}


bool
FreeStateMachine::finished(ExecContext* xc) const {
    return true;
}

Fault
FreeStateMachine::transit(ExecContext* xc, PacketPtr pkt) {
    return NoFault;
}



} // namespace RiscvcapstoneISA
} // namespace gem5
