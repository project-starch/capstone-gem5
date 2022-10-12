#ifndef RISCV_NOMMU_SYSCALL_EMUL_H
#define RISCV_NOMMU_SYSCALL_EMUL_H

#include "cpu/thread_context.hh"
#include "sim/system.hh"
#include "sim/syscall_return.hh"
#include "sim/syscall_desc.hh"

namespace gem5::RiscvISA {
    SyscallReturn
        notifymallocFunc(SyscallDesc* desc, ThreadContext* tc,
                uint64_t addr, uint64_t size);

    SyscallReturn
        notifyfreeFunc(SyscallDesc* desc, ThreadContext* tc,
                uint64_t addr);
}

#endif

