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

#ifndef __ARCH_RISCV_STATIC_INST_HH__
#define __ARCH_RISCV_STATIC_INST_HH__

#include <string>

#include "arch/riscvcapstone/node_controller.hh"
#include "arch/riscvcapstone/pcstate.hh"
#include "arch/riscvcapstone/types.hh"
#include "base/types.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "mem/packet.hh"

namespace gem5
{

namespace RiscvcapstoneISA
{
    namespace o3 {
        class NodeCommand;
    }

struct InstStateMachine {
    virtual void setup(ExecContext* xc) = 0;
    virtual bool finished(ExecContext* xc) const = 0;
    virtual Fault transit(ExecContext* xc, PacketPtr pkt) = 0;
    virtual Tick atomicExec(ExecContext* xc) = 0;


    // o3
    //virtual NodeControllerCommandPtr getNodeControllerReadCmd() = 0;
    //virtual NodeControllerCommandPtr getNodeControllerWriteCmd() = 0;
};


struct DummyInstStateMachine : InstStateMachine {
    void setup(ExecContext* xc) override {}

    bool finished(ExecContext* xc) const override {
        return true;
    }
    
    Fault transit(ExecContext* xc, PacketPtr pkt) override {
        return NoFault;
    }

    Tick atomicExec(ExecContext* xc) override {
        return 0;
    }

    //NodeControllerCommandPtr getNodeControllerReadCmd() override {
        //return nullptr;
    //}

    //NodeControllerCommandPtr getNodeControllerWriteCmd() override {
        //return nullptr;
    //}
};

typedef std::shared_ptr<InstStateMachine> InstStateMachinePtr;



/**
 * Base class for all RISC-V static instructions.
 */
class RiscvStaticInst : public StaticInst
{
  protected:
    RiscvStaticInst(const char *_mnemonic, ExtMachInst _machInst,
            OpClass __opClass) :
        StaticInst(_mnemonic, __opClass), machInst(_machInst)
    {}

  public:
    bool hasNodeOp = false;
    bool hasTagReq = false;
    bool hasNodeWB = false;
      
    ExtMachInst machInst;

    void
    advancePC(PCStateBase &pc) const override
    {
        pc.as<PCState>().advance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        pc.advance();
        tc->pcState(pc);
    }

    std::unique_ptr<PCStateBase>
    buildRetPC(const PCStateBase &cur_pc,
            const PCStateBase &call_pc) const override
    {
        PCStateBase *ret_pc_ptr = call_pc.clone();
        auto &ret_pc = ret_pc_ptr->as<PCState>();
        ret_pc.advance();
        return std::unique_ptr<PCStateBase>{ret_pc_ptr};
    }

    size_t
    asBytes(void *buf, size_t size) override
    {
        return simpleAsBytes(buf, size, machInst);
    }

    virtual InstStateMachinePtr getStateMachine(ExecContext* xc) const {
        return std::make_shared<DummyInstStateMachine>();
    }

    // FIXME: here "mem" actually only includes the node cache
    bool pendingMem(InstStateMachinePtr sm, ExecContext* xc) const {
        return !sm->finished(xc);
    }// returns whether the operation has a pending memory access

    virtual Fault handleMemResp(InstStateMachinePtr sm, ExecContext* xc, PacketPtr pkt) {
        return sm->transit(xc, pkt);
    }

    virtual Addr getAddr(ExecContext *, Trace::InstRecord *) const {
        return 0;
    }

    virtual Fault initiateAcc(ExecContext *xc, Trace::InstRecord *traceData) const {
        return NoFault;
    }

    virtual Fault completeAcc(PacketPtr pkt, ExecContext *xc, Trace::InstRecord *traceData) const {
        return NoFault;
    }
};

/**
 * Base class for all RISC-V Macroops
 */
class RiscvMacroInst : public RiscvStaticInst
{
  protected:
    std::vector<StaticInstPtr> microops;

    RiscvMacroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            RiscvStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMacroop] = true;
    }

    ~RiscvMacroInst() { microops.clear(); }

    StaticInstPtr
    fetchMicroop(MicroPC upc) const override
    {
        return microops[upc];
    }

    Fault
    initiateAcc(ExecContext *xc, Trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    completeAcc(PacketPtr pkt, ExecContext *xc,
                Trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    execute(ExecContext *xc, Trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }
};

/**
 * Base class for all RISC-V Microops
 */
class RiscvMicroInst : public RiscvStaticInst
{
  protected:
    RiscvMicroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            RiscvStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMicroop] = true;
    }

    void advancePC(PCStateBase &pcState) const override;
    void advancePC(ThreadContext *tc) const override;
};

} // namespace RiscvcapstoneISA
} // namespace gem5

#endif // __ARCH_RISCV_STATIC_INST_HH__
