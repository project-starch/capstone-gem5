#include "arch/riscvnommu/linux/syscall_emul.hh"

namespace gem5::RiscvnommuISA {


SyscallReturn
notifymallocFunc(SyscallDesc* desc, ThreadContext* tc,
        uint64_t addr, uint64_t size) {
    return addr;
}

SyscallReturn
notifyfreeFunc(SyscallDesc* desc, ThreadContext* tc,
        uint64_t addr) {
    return SyscallReturn();
}

// FIXME: the return value ovewrites registers. Needs to clean up the capabilities inside

}

