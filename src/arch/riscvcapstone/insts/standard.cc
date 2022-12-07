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
#include "arch/riscvcapstone/atomic_ncache_cpu.hh"
#include "cpu/exec_context.hh"
#include "cpu/simple/exec_context.hh"
#include "cpu/static_inst.hh"
#include "debug/CapstoneNodeOps.hh"

namespace gem5
{

namespace RiscvcapstoneISA
{
std::string
RevokeClass::generateDisassembly(Addr pc,
                                const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(0));
    return ss.str();
}

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
    RegVal num = xc->tcBase()->readIntReg(SyscallNumReg);
    switch(num) {
        case 3000: // malloc
            return std::make_shared<MallocStateMachine>(
                    (Addr)xc->tcBase()->getReg(RegId(IntRegClass, ReturnValueReg)),
                    (uint64_t)xc->tcBase()->getReg(RegId(IntRegClass, ReturnValueReg + 1)));
        case 3001:
            return std::make_shared<FreeStateMachine>(
                    CapLoc::makeReg(xc->tcBase()->threadId(), ArgumentRegs[0]));
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
    
    NodeControllerAllocate* cmd = new NodeControllerAllocate(NODE_ID_INVALID);

    TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "non ncache-cpu unsupported.");
    cpu->sendNCacheCommand(cmd);

    state = MALLOC_ALLOC_NODE;
}

bool
MallocStateMachine::finished(ExecContext* xc) const {
    return state == MALLOC_DONE;
}

// Note that this is executed before the actual syscall handler
Fault
MallocStateMachine::transit(ExecContext* xc, PacketPtr pkt) {
    NodeID node_id = pkt->getRaw<NodeID>();
    DPRINTF(CapstoneNodeOps, "Allocated node %u\n", node_id);

    TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "non ncache-cpu unsupported.");

    cpu->node_controller->addCapTrack(CapLoc::makeReg(xc->tcBase()->threadId(), ReturnValueReg), 
            node_id);
    DPRINTF(CapstoneNodeOps, "Associated node %llu with addr range (0x%llx, 0x%llx)\n", 
            node_id,
            addr, (Addr)(addr + size));
    cpu->node_controller->node2Obj[node_id] = SimpleAddrRange(addr, (Addr)(addr + size));
    
    state = MALLOC_DONE;

    return NoFault;
}

void
FreeStateMachine::setup(ExecContext* xc) {

    TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "non ncache-cpu unsupported.");
    
    NodeID node_id = cpu->node_controller->queryCapTrack(loc);
    if(node_id == NODE_ID_INVALID) {
        DPRINTF(CapstoneNodeOps, "warning: no node associated with the location to free\n");
        state = FREE_DONE;
        return;
    }
    DPRINTF(CapstoneNodeOps, "Free node %u\n", node_id);
    //panic_if(node_id == NODE_ID_INVALID, "attempted to revoke an invalid node id");

    NodeControllerRevoke* cmd = new NodeControllerRevoke(node_id);
    cpu->sendNCacheCommand(cmd);

    state = FREE_FREE_NODE;
}


bool
FreeStateMachine::finished(ExecContext* xc) const {
    return state == FREE_DONE;
}

Fault
FreeStateMachine::transit(ExecContext* xc, PacketPtr pkt) {
    state = FREE_DONE;
    return NoFault;
}

Tick
MallocStateMachine::atomicExec(ExecContext* xc) {
    AtomicSimpleNCacheCPU* cpu = 
        dynamic_cast<AtomicSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "only atomic ncache cpu supports atomic execution");

    PacketPtr pkt = cpu->sendNCacheCommandAtomic(
            new NodeControllerAllocate(NODE_ID_INVALID));
    NodeID node_id = pkt->getRaw<NodeID>();
    cpu->node_controller->addCapTrack(
            CapLoc::makeReg(xc->tcBase()->threadId(), ReturnValueReg), 
            node_id);
    DPRINTF(CapstoneNodeOps, "Associated node %llu with addr range (0x%llx, 0x%llx)\n", 
            node_id,
            addr, (Addr)(addr + size));
    cpu->node_controller->node2Obj[node_id] = SimpleAddrRange(addr, (Addr)(addr + size));

    delete pkt;

    return 0;
}

Tick
FreeStateMachine::atomicExec(ExecContext* xc) {
    AtomicSimpleNCacheCPU* cpu = 
        dynamic_cast<AtomicSimpleNCacheCPU*>(xc->tcBase()->getCpuPtr());
    panic_if(cpu == NULL, "only atomic ncache cpu supports atomic execution");

    NodeID node_id = cpu->node_controller->queryCapTrack(loc);
    if(node_id == NODE_ID_INVALID) {
        DPRINTF(CapstoneNodeOps, "warning: no node associated with the location to free %llx\n",
                xc->tcBase()->readIntReg(ArgumentRegs[0]));
        return 0;
    }

    PacketPtr pkt = cpu->sendNCacheCommandAtomic(
            new NodeControllerRevoke(node_id));
    delete pkt;

    return 0;
}


//NodeControllerCommandPtr
//FreeStateMachine::getNodeControllerReadCmd(ExecContext* xc) {
    
//}


} // namespace RiscvcapstoneISA
} // namespace gem5
