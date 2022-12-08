#include "arch/riscvcapstone/o3/ncq.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"



namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


NCQ::NCQ(CPU* cpu, int queue_size, int thread_num) : 
    cpu(cpu),
    queueSize(queue_size),
    threadNum(thread_num),
    activeThreads(nullptr) {
    
    threads.reserve(thread_num);
    for(int i = 0; i < thread_num; i ++) {
        threads.emplace_back(static_cast<ThreadID>(i), queue_size);
    }
}

void 
NCQ::insertInstruction(const DynInstPtr& inst) {
    assert(inst->threadNumber >= 0 && inst->threadNumber < threadNum);
    threads[inst->threadNumber].insertInstruction(inst);
}

void
NCQ::tick() {
    for(auto& t : threads)
        t.tick();
}

Fault
NCQ::pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd) {
    assert(inst->threadNumber >= 0 && inst->threadNumber < threadNum);
    return threads[inst->threadNumber].pushCommand(inst, cmd);
}

bool
NCQ::isFull(ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    return threads[thread_id].isFull();
}

Fault
NCQ::executeNodeOp(const DynInstPtr& inst) {
    Fault fault = inst->initiateNodeAcc();
    return NoFault;
}

void
NCQ::commitBefore(InstSeqNum seq_num, ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    threads[thread_id].commitBefore(seq_num);
}

void
NCQ::writebackCommands(ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    threads[thread_id].writebackCommands();
}

void
NCQ::writebackCommands() {
    for(ThreadID& thread_id : *activeThreads) {
        assert(thread_id >= 0 && thread_id < threadNum);
        threads[thread_id].writebackCommands();
    }
}

}
}
}

