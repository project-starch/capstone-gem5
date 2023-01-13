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
BaseTagController::setTag(const DynInstPtr& inst, Addr addr, NodeID tag) {
    assert(inst->tqIdx >= 0);

    if(!aligned(addr)) {
        assert(!tag);
        // cannot tag unaligned addr
        return;
    }

    inst->tqIt->ops.push_back(TagOp {.addr = addr, .tagSet = tag });
}

void
BaseTagController::commitBefore(const InstSeqNum& seq_num, ThreadID thread_id) {
    TagQueue& tag_queue = tagQueues[thread_id];

    for(TagQueue::iterator it = tag_queue.begin();
        it != tag_queue.end() && it->inst->seqNum <= seq_num; ++ it) {
        it->canWB = true;
    }
}

void
BaseTagController::squash(const InstSeqNum& seq_num, ThreadID thread_id) {
    TagQueue& tag_queue = tagQueues[thread_id];

    while(!tag_queue.empty() && tag_queue.back().inst->seqNum > seq_num){
        DPRINTF(TagController, "Instruction %llu at %llx squashed in the tag controller\n",
                tag_queue.back().inst->seqNum,
                tag_queue.back().inst->pcState().instAddr());
        tag_queue.back().inst->setSquashed();
        tag_queue.back().clear();
        tag_queue.pop_back();
    }
}


NodeID
BaseTagController::getTag(const DynInstPtr& inst, Addr addr,
        bool& delayed) {
    assert(inst->tqIdx >= 0);

    if(!aligned(addr)) {
        DPRINTF(TagController, "Unaligned address %llx, hence tag is 0\n",
            addr);
        delayed = false;
        return NODE_ID_INVALID;
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
                tq_it->front().clear(), tq_it->pop_front());
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
    regTagMaps(thread_count, RegTagMap(REG_N))
{
    for(int i = 0; i < thread_count; i ++)
        for(int j = 0; j < REG_N; j ++)
            regTagMaps[i][j] = NODE_ID_INVALID;
}


NodeID
MockTagController::getCommittedTag(const DynInstPtr& inst,
        Addr addr, bool& delayed) {
    delayed = false;
    // check the alignment
    if(!aligned(addr)) {
        return NODE_ID_INVALID;
    }
    auto it = taggedAddrs.find(addr);
    if(it == taggedAddrs.end())
        return NODE_ID_INVALID;
    return it->second;
}


bool
MockTagController::writebackTagOp(DynInstPtr& inst, TagOp& tag_op) {
    assert(aligned(tag_op.addr));
    if(tag_op.tagSet != NODE_ID_INVALID) {
        taggedAddrs[tag_op.addr] = tag_op.tagSet;
    } else {
        taggedAddrs.erase(tag_op.addr);
    }
    return true; // 0 latency for mock tag controller 
                 // to update the state
}


NodeID
MockTagController::getRegTag(RegIndex reg_idx, ThreadID thread_id) const {
    return regTagMaps[thread_id][reg_idx];
}

void
MockTagController::setRegTag(RegIndex reg_idx, NodeID tag, ThreadID thread_id) {
    regTagMaps[thread_id][reg_idx] = tag;
}

void
MockTagController::allocObject(NodeID node_id, const SimpleAddrRange& range, ThreadID thread_id) {
    objectMap[node_id] = range;
}

void
MockTagController::freeObject(Addr addr, ThreadID thread_id) {
    // nothing needs to be done
}

SimpleAddrRange
MockTagController::getObject(NodeID node_id) {
    return objectMap[node_id];
}

}
}
}

