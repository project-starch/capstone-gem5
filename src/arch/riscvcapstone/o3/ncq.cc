#include "arch/riscvcapstone/o3/ncq.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"



namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


NCQ::NCQ(int queue_size, int thread_num) : 
    queueSize(queue_size),
    threadNum(thread_num) {
    
    threads.reserve(thread_num);
    for(int i = 0; i < thread_num; i ++) {
        threads.emplace_back(queue_size);
    }
}

void 
NCQ::insertInstruction(const DynInstPtr& inst) {
    assert(inst->threadNumber >= 0 && inst->threadNumber < threadNum);
    threads[inst->threadNumber].insertInstruction(inst);
}

void
NCQ::tick() {
    for(auto t : threads)
        t.tick();
}



Fault
NCQ::pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd) {
    assert(inst->threadNumber >= 0 && inst->threadNumber < threadNum);
    return threads[inst->threadNumber].pushCommand(inst, cmd);
}


}
}
}

