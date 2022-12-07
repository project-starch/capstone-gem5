#include "arch/riscvcapstone/o3/ncq_unit.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


NCQUnit::NCQUnit(ThreadID thread_id, int queue_size) :
    threadId(thread_id),
    ncQueue(queue_size),
    queueSize(queue_size)
{
}

void
NCQUnit::insertInstruction(const DynInstPtr& inst) {
    assert(!ncQueue.full());
    ncQueue.advance_tail();
    ncQueue.back() = NCQEntry(inst);
    assert(!ncQueue.empty());
}

void
NCQUnit::tick() {
    // for testing
    if(!ncQueue.empty())
        ncQueue.pop_front();
    assert(!ncQueue.full());
}

Fault
NCQUnit::pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd) {
    return NoFault;
}

bool
NCQUnit::isFull() {
    return ncQueue.full();
}

}
}
}


