#include "arch/riscvcapstone/o3/tag_controller.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

void
MockTagController::setTag(const DynInstPtr& inst, Addr addr, bool tag) {
    if(!aligned(addr)) {
        assert(!tag);
        // cannot tag unaligned addr
        return;
    }
    tagQueue.push_front(TagEntry {
        inst, addr, tag
    });
}

bool
MockTagController::getTag(Addr addr) const {
    if(!aligned(addr)) {
        return false;
    }

    for(auto& tag_entry : tagQueue) {
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
MockTagController::commitBefore(InstSeqNum seq_num) {
    for(std::list<TagEntry>::iterator it = tagQueue.begin();
        it != tagQueue.end(); ) {
        if(it->inst->seqNum <= seq_num) {
            commitTag(*it);
            it = tagQueue.erase(it);
        } else {
            ++ it;
        }

    }
}

}
}
}
