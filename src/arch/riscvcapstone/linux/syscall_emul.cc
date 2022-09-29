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

