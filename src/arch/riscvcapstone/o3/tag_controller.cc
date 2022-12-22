#include "arch/riscvcapstone/o3/tag_controller.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

MockTagController::MockTagController(int thread_count):
    threadCount(thread_count) {
    tagQueues.reserve(thread_count);
    for(int i = 0; i < thread_count; i ++) {
        tagQueues.emplace_back();
    }
}

void
MockTagController::setTag(const DynInstPtr& inst, Addr addr, bool tag, 
        ThreadID thread_id) {
    if(!aligned(addr)) {
        assert(!tag);
        // cannot tag unaligned addr
        return;
    }
    tagQueues[thread_id].push_front(TagEntry {
        inst, addr, tag
    });
}

bool
MockTagController::getTag(Addr addr, ThreadID thread_id) const {
    if(!aligned(addr)) {
        return false;
    }

    for(auto& tag_entry : tagQueues[thread_id]) {
        if(tag_entry.addr == addr) {
            return tag_entry.tagSet;
        }
    }
    return getCommittedTag(addr);
}

bool
MockTagController::getCommittedTag(Addr addr) const {
    // check the alignment
    if(!aligned(addr)) {
        return false;
    }
    return taggedAddrs.find(addr) != taggedAddrs.end();
}

void
MockTagController::commitBefore(InstSeqNum seq_num, ThreadID thread_id) {
    TagQueue& tag_queue = tagQueues[thread_id];

    for(TagQueue::iterator it = tag_queue.begin();
        it != tag_queue.end(); ) {
        if(it->inst->seqNum <= seq_num) {
            commitTag(*it);
            it = tag_queue.erase(it);
        } else {
            ++ it;
        }

    }
}

}
}
}
