#include "arch/riscvcapstone/linux/syscall_emul.hh"
#include "debug/CapstoneAlloc.hh"
#include "arch/riscvcapstone/ncache_cpu.hh"

namespace gem5::RiscvcapstoneISA {
    SyscallReturn
        notifymallocFunc(SyscallDesc* desc, ThreadContext* tc,
                uint64_t addr, uint64_t size) {
            DPRINTF(CapstoneAlloc, "malloc: %llx, %llx\n", addr, size);

            TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(tc->getCpuPtr());
            if(cpu && cpu->node_controller) {
                cpu->node_controller->allocObject(AddrRange((Addr)addr, (Addr)(addr + size)));
            } else {
                DPRINTF(CapstoneAlloc, "malloc: warning! cpu is not TimingSimpleNCacheCPU or does not have a node controller set!\n");
            }
            return 0;
        }

    SyscallReturn
        notifyfreeFunc(SyscallDesc* desc, ThreadContext* tc,
                uint64_t addr) {
            DPRINTF(CapstoneAlloc, "free: %llx\n", addr);

            TimingSimpleNCacheCPU* cpu = dynamic_cast<TimingSimpleNCacheCPU*>(tc->getCpuPtr());
            if(cpu && cpu->node_controller) {
                cpu->node_controller->freeObject((Addr)addr);
            } else{
                DPRINTF(CapstoneAlloc, "free: warning! cpu is not TimingSimpleNCacheCPU or does not have a node controller set!\n");
            }
            return 0;
        }

}

