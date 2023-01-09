#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "arch/riscvcapstone/o3/tag_controller.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/iew.hh"
#include "debug/TagController.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

BaseTagController::BaseTagController(int thread_count, int queue_size):
    threadCount(thread_count),
    queueSize(queue_size) {
    tagQueues.reserve(thread_count);
    for(int i = 0; i < thread_count; i ++) {
        tagQueues.emplace_back(queue_size);
    }
}

void
BaseTagController::setTag(const DynInstPtr& inst, Addr addr, bool tag) {
    assert(inst->tqIdx >= 0);

    if(!aligned(addr)) {
        assert(!tag);
        // cannot tag unaligned addr
        return;
    }

    inst->tqIt->ops.push_back(TagOp {.addr = addr, .tagSet = tag });
}

void
BaseTagController::commitBefore(InstSeqNum seq_num, ThreadID thread_id) {
    TagQueue& tag_queue = tagQueues[thread_id];

    for(TagQueue::iterator it = tag_queue.begin();
        it != tag_queue.end() && it->inst->seqNum <= seq_num; ++ it) {
        it->canWB = true;
    }
}


bool
BaseTagController::getTag(const DynInstPtr& inst, Addr addr,
        bool& delayed) {
    DPRINTF(TagController, "Tag controller get tag at %llx [sn:%u]\n",
            addr, inst->seqNum);

    assert(inst->tqIdx >= 0);

    if(!aligned(addr)) {
        DPRINTF(TagController, "Unaligned address %llx, hence tag is 0\n",
            addr);
        delayed = false;
        return false;
    }

    ThreadID thread_id = inst->threadNumber;

    TagQueue& tag_queue = tagQueues[thread_id];

    // need to start from the most recent
    for(auto it = inst->tqIt;; -- it) {
        for(auto op_it = it->ops.rbegin(); op_it != it->ops.rend(); ++ op_it) {
            if(op_it->addr == addr) {
                delayed = false;
                return op_it->tagSet;
            }
        }
        if(it == tag_queue.begin())
            break;
    }
    return getCommittedTag(inst, addr, delayed);
}

void
BaseTagController::insertInstruction(const DynInstPtr& inst) {
    DPRINTF(TagController, "Tag controller insert inst [sn:%u]\n",
            inst->seqNum);

    ThreadID thread_id = inst->threadNumber;
    assert(thread_id >= 0 && thread_id < threadCount);
    TagQueue& tq = tagQueues[thread_id];
    assert(!tq.full());
    tq.advance_tail();
    tq.back() = TagEntry { 
        .inst = inst,
        .canWB = false
    };

    inst->tqIdx = tq.tail();
    inst->tqIt = tq.end() - 1;
}

void
BaseTagController::writeback() {
    // TODO: multithreaded scenario?
    for(auto tq_it = tagQueues.begin();
            tq_it != tagQueues.end();
            ++ tq_it) {
        for(; !tq_it->empty() && tq_it->front().canWB &&
                writebackTagEntry(tq_it->front());
                tq_it->pop_front());
    }
}

bool
BaseTagController::writebackTagEntry(TagEntry& tag_entry) {
    for(auto it = tag_entry.ops.begin(); it != tag_entry.ops.end() &&
            writebackTagOp(tag_entry.inst, *it);
            it = tag_entry.ops.erase(it));
    return tag_entry.ops.empty();
}

MockTagController::MockTagController(int thread_count, int queue_size):
    BaseTagController(thread_count, queue_size),
    regTagMaps(thread_count, RegTagMap(REG_N, false))
{
}


bool
MockTagController::getCommittedTag(const DynInstPtr& inst,
        Addr addr, bool& delayed) {
    delayed = false;
    // check the alignment
    if(!aligned(addr)) {
        return false;
    }
    return taggedAddrs.find(addr) != taggedAddrs.end();
}


bool
MockTagController::writebackTagOp(DynInstPtr& inst, TagOp& tag_op) {
    assert(aligned(tag_op.addr));
    if(tag_op.tagSet) {
        taggedAddrs.insert(tag_op.addr);
    } else {
        taggedAddrs.erase(tag_op.addr);
    }
    return true; // 0 latency for mock tag controller 
                 // to update the state
}


bool
MockTagController::getRegTag(RegIndex reg_idx, ThreadID thread_id) const {
    return regTagMaps[reg_idx][thread_id];
}

void
MockTagController::setRegTag(RegIndex reg_idx, bool tag, ThreadID thread_id) {
    regTagMaps[reg_idx][thread_id] = tag;
}

}
}
}

