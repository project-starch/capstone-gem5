#include "arch/riscvcapstone/linux/syscall_emul.hh"
#include "debug/CapstoneAlloc.hh"
#include "arch/riscvcapstone/o3/cpu.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/typing.hh"
//#include "arch/riscvcapstone/o3/cpu.hh"

namespace gem5::RiscvcapstoneISA {


SyscallReturn
notifymallocFunc(SyscallDesc* desc, ThreadContext* tc,
        uint64_t addr, uint64_t size) {
    DPRINTF(CapstoneAlloc, "malloc: %llx, %llu\n", addr, size);

    using o3::CPU;

    CPU* cpu = 
        dynamic_cast<CPU*>(tc->getCpuPtr());
 //   o3::CPU* o3_cpu = dynamic_cast<o3::CPU*>(tc->getCpuPtr());    
    // TODO: there should be more graceful solutions
    if(cpu) {
        cpu->allocObject(SimpleAddrRange((Addr)addr, (Addr)(addr + size)));
    //} else if(o3_cpu) {
        //o3_cpu->getNodeController()->
            //allocObject(SimpleAddrRange((Addr)addr, (Addr)(addr + size)));
    } else {
        DPRINTFN("malloc: warning! cpu does not have a node controller!\n");
    }
    return addr;
}

SyscallReturn
notifyfreeFunc(SyscallDesc* desc, ThreadContext* tc,
        uint64_t addr) {
    DPRINTF(CapstoneAlloc, "free: %llx\n", addr);

    using o3::CPU;

    CPU* cpu = 
        dynamic_cast<CPU*>(tc->getCpuPtr());
    //o3::CPU* o3_cpu = dynamic_cast<o3::CPU*>(tc->getCpuPtr());    
    if(cpu) {
        cpu->freeObject((Addr)addr);
    //} else if(o3_cpu) {
        //o3_cpu->getNodeController()->freeObject((Addr)addr);
    } else {
        DPRINTFN("free: warning! cpu does not have a node controller!\n");
    }
    return SyscallReturn();
}

// FIXME: the return value ovewrites registers. Needs to clean up the capabilities inside

}

