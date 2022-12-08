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

    inst->ncqIdx = ncQueue.tail();
    inst->ncqIt = ncQueue.end() - 1;
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
    assert(inst->ncqIdx != -1); // inst has been inserted to this queue
    NCQEntry& ncq_entry = *(inst->ncqIt);
    assert(ncq_entry.inst->seqNum == inst->seqNum); // indeed the same inst in the entry

    ncq_entry.commands.push_back(cmd);

    return NoFault;
}

bool
NCQUnit::isFull() {
    return ncQueue.full();
}

void
NCQUnit::commitBefore(InstSeqNum seq_num) {
    for(NCQIterator it = ncQueue.begin(); 
            it != ncQueue.end() && it->inst->seqNum <= seq_num;
            ++ it) {
        it->canWB = true;
    }
    // TODO
}

void
NCQUnit::writebackCommands(){
    // TODO
}


void
NCQUnit::completeCommand(NCQIterator cmd_it){
    // TODO
}


}
}
}


