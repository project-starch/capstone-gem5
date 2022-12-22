#include "mem/packet.hh"
#include "arch/riscvcapstone/o3/tag_controller.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

BaseTagController::BaseTagController(int thread_count):
    threadCount(thread_count) {
    tagQueues.reserve(thread_count);
    for(int i = 0; i < thread_count; i ++) {
        tagQueues.emplace_back();
    }
}

void
BaseTagController::setTag(const DynInstPtr& inst, Addr addr, bool tag, 
        ThreadID thread_id) {
    if(!aligned(addr)) {
        assert(!tag);
        // cannot tag unaligned addr
        return;
    }
    tagQueues[thread_id].push_back(TagEntry {
        inst, addr, tag
    });
}

void
BaseTagController::commitBefore(InstSeqNum seq_num, ThreadID thread_id) {
    TagQueue& tag_queue = tagQueues[thread_id];

    for(TagQueue::iterator it = tag_queue.begin();
        it != tag_queue.end(); ) {
        if(it->inst->seqNum <= seq_num) {
            commitTag(*it, thread_id);
            it = tag_queue.erase(it);
        } else {
            ++ it;
        }

    }
}


bool
BaseTagController::getTag(const DynInstPtr& inst, Addr addr,
        ThreadID thread_id, bool& delayed) {
    if(!aligned(addr)) {
        delayed = false;
        return false;
    }

    TagQueue& tag_queue = tagQueues[thread_id];

    // need to start from the most recent
    for(TagQueue::const_reverse_iterator it = tag_queue.rbegin();
            it != tag_queue.rend();
            ++ it) {
        if(it->addr == addr) {
            delayed = false;
            return it->tagSet;
        }
    }
    return getCommittedTag(inst, addr, delayed);
}

MockTagController::MockTagController(int thread_count):
    BaseTagController(thread_count)
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


void
MockTagController::commitTag(const TagEntry& tag_entry, 
        ThreadID thread_id) {
    assert(aligned(tag_entry.addr));
    if(tag_entry.tagSet) {
        taggedAddrs.insert(tag_entry.addr);
    } else {
        taggedAddrs.erase(tag_entry.addr);
    }
}


MemoryTagController::MemoryTagController(CPU* cpu, 
        int thread_count, int tcache_ports_count) :
            BaseTagController(thread_count),
            cpu(cpu),
            tcachePort(this, cpu, tcache_ports_count) {
    wbQueues.reserve(thread_count);
    for(int i = 0; i < thread_count; i ++){
        wbQueues.emplace_back();
    }
}

void
MemoryTagController::commitTag(const TagEntry& tag_entry, 
        ThreadID thread_id) {
    wbQueues[thread_id].push_back(tag_entry);
}

bool
MemoryTagController::getCommittedTag(const DynInstPtr& inst,
        Addr addr, bool& delayed) {
    delayed = true;

    Addr tag_addr = getTagAddr(addr);

    RequestPtr req = std::make_shared<Request>();
    req->requestorId(inst->requestorId());
    req->setPaddr(tag_addr);

    PacketPtr pkt = Packet::createRead(req);
    pkt->setSize(1);
    pkt->allocate();

    tcachePort.trySendPacket(pkt);

    assert(ongoingRequests.find(pkt->id) == ongoingRequests.end());
    ongoingRequests[pkt->id] = TagCacheRequest {
        .entry = TagEntry {
            .inst = inst,
            .addr = addr,
            .tagSet = false
        },
        .isWrite = false
    };

    return false; // does not actually matter what we return since
                  // it is delayed
}

void
MemoryTagController::tick() {
    tcachePort.tick();
}


void
MemoryTagController::writeback() {
    // write back to cache the oldest first
    // TODO: consider reordering
    // TODO: consider ordering of threads

    for(auto& wb_queue : wbQueues) {
        for(TagQueue::iterator it = wb_queue.begin();
            it != wb_queue.end() && tcachePort.canSend();
            it = wb_queue.erase(it)) {
            writeback(*it);
        }
    }
}

void
MemoryTagController::writeback(const TagEntry& tag_entry) {
    Addr addr = getTagAddr(tag_entry.addr);

    RequestPtr req = std::make_shared<Request>();
    req->requestorId(tag_entry.inst->requestorId());
    req->setPaddr(addr);

    PacketPtr pkt = Packet::createWrite(req);
    pkt->setSize(1);
    pkt->allocate();
    *(pkt->getPtr<uint8_t>()) = tag_entry.tagSet ? 1 : 0;

    tcachePort.trySendPacket(pkt);

    assert(ongoingRequests.find(pkt->id) == ongoingRequests.end());
    ongoingRequests[pkt->id] = TagCacheRequest {
        .entry = tag_entry,
        .isWrite = true
    };
}

bool
MemoryTagController::handleResp(PacketPtr pkt) {
    auto it = ongoingRequests.find(pkt->id);
    assert(it != ongoingRequests.end());
    if(it->second.isWrite) {
        // TODO: potentially mark the instruction as complete
        // compare with store completion
    } else{
        // TODO: call the instruction's complete tag load
    }
}

void
MemoryTagController::TagCachePort::trySendPacket(PacketPtr pkt) {
    assert(canSend());
    ++ portsUsed;
    if(!sendTimingReq(pkt)) {
        blocked = true;
        blockedPacket = pkt;
    }
}


MemoryTagController::
TagCachePort::TagCachePort(MemoryTagController* owner,
        CPU* cpu, int ports_count):
    RequestPort(cpu->name() + ".tcache_port", cpu),
    owner(owner), cpu(cpu),
    portsCount(ports_count)
{}

bool
MemoryTagController::TagCachePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResp(pkt);
}

void
MemoryTagController::TagCachePort::recvReqRetry() {
    assert(blocked);
    blocked = false;
    trySendPacket(blockedPacket);
}


}
}
}

