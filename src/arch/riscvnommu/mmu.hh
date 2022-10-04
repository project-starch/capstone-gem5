/*
 * Copyright (c) 2020 ARM Limited
 * All rights reserved
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

#ifndef __ARCH_RISCV_MMU_HH__
#define __ARCH_RISCV_MMU_HH__

#include "arch/generic/mmu.hh"
#include "arch/riscvnommu/isa.hh"
#include "arch/riscvnommu/page_size.hh"
#include "arch/riscvnommu/pma_checker.hh"
#include "arch/riscvnommu/tlb.hh"

#include "params/RiscvMMU.hh"

#include "base/trace.hh"
#include "debug/CapstoneMem.hh"

namespace gem5
{

namespace RiscvnommuISA {

class MMU : public BaseMMU
{
  public:
    MMU(const RiscvMMUParams &p)
      : BaseMMU(p)
    {}

    TranslationGenPtr
    translateFunctional(Addr start, Addr size, ThreadContext *tc,
            Mode mode, Request::Flags flags) override
    {
        return TranslationGenPtr(new MMUTranslationGen(
                PageBytes, start, size, tc, this, mode, flags));
    }

    PrivilegeMode
    getMemPriv(ThreadContext *tc, BaseMMU::Mode mode)
    {
        return PrivilegeMode::PRV_U;
    }


    Fault translateAtomic(const RequestPtr& req, ThreadContext* tc, Mode mode) override {
        DPRINTF(CapstoneMem, "translate (atomic) %llx\n", req->getVaddr());
        req->setPaddr(req->getVaddr()); // simply pass through
        return NoFault;
    }

    void translateTiming(const RequestPtr& req, ThreadContext* tc,
            Translation* translation, Mode mode) override {
        DPRINTF(CapstoneMem, "translate %llx\n", req->getVaddr());
        req->setPaddr(req->getVaddr()); // simply pass through
        translation->finish(NoFault, req, tc, mode);
    }

    Fault translateFunctional(const RequestPtr& req, ThreadContext* tc, Mode mode) override {
        req->setPaddr(req->getVaddr()); // simply pass through

        return NoFault;
    }

    void flushAll() override {} 

    Fault finalizePhysical(const RequestPtr &req, ThreadContext *tc, Mode mode) const override {
        return NoFault;
    }

    void
    takeOverFrom(BaseMMU *old_mmu) override {}

};

} // namespace RiscvnommuISA
} // namespace gem5

#endif  // __ARCH_RISCV_MMU_HH__
