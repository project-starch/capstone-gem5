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
        tag_queue.back().inst->setSquashed();
        tag_queue.back().clear();
        tag_queue.pop_back();
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
    BaseTagController(thread_count, queue_size)
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

MemoryTagController::MemoryTagController(CPU* cpu, IEW* iew,
        int thread_count, int tcache_ports_count, int queue_size) :
            BaseTagController(thread_count, queue_size),
            cpu(cpu), iew(iew),
            tcachePort(this, cpu, tcache_ports_count){
}

bool
MemoryTagController::getCommittedTag(const DynInstPtr& inst,
        Addr addr, bool& delayed) {
    DPRINTF(TagController, "tag controller sending tag query for inst %u\n",
            inst->seqNum);
    delayed = true;

    Addr tag_addr = getTagAddr(addr);

    RequestPtr req = std::make_shared<Request>();
    req->requestorId(inst->requestorId());
    req->setPaddr(tag_addr);

    PacketPtr pkt = Packet::createRead(req);
    pkt->setSize(1);
    pkt->allocate();
    
    TagCacheRequest tcache_req = {
        .inst = inst,
        .op = TagOp {
            .addr = addr,
            .tagSet = false
        },
        .isWrite = false
    };

    if(!tcachePort.canSend()) {
        blockedRequests.push(std::make_pair(pkt, tcache_req));
    } else {
        tcachePort.trySendPacket(pkt);

        assert(ongoingRequests.find(pkt->id) == ongoingRequests.end());
        ongoingRequests[pkt->id] = tcache_req;
    }

    return false; // does not actually matter what we return since
                  // it is delayed
}

void
MemoryTagController::tick() {
    tcachePort.tick();
}

bool
MemoryTagController::writebackTagOp(DynInstPtr& inst, TagOp& tag_op) {
    if(!tcachePort.canSend())
        return false;

    DPRINTF(TagController, "Tag controller writebackTagOp for inst %u\n",
            inst->seqNum);

    Addr addr = getTagAddr(tag_op.addr);

    RequestPtr req = std::make_shared<Request>();
    req->requestorId(inst->requestorId());
    req->setPaddr(addr);

    PacketPtr pkt = Packet::createWrite(req);
    pkt->setSize(1);
    pkt->allocate();
    *(pkt->getPtr<uint8_t>()) = tag_op.tagSet ? 1 : 0;

    tcachePort.trySendPacket(pkt);

    assert(ongoingRequests.find(pkt->id) == ongoingRequests.end());
    ongoingRequests[pkt->id] = TagCacheRequest {
        .inst = inst,
        .op = tag_op,
        .isWrite = true
    };

    return true;
}

bool
MemoryTagController::handleResp(PacketPtr pkt) {
    DPRINTF(TagController, "tag controller received response\n");
    auto it = ongoingRequests.find(pkt->id);
    assert(it != ongoingRequests.end());

    TagCacheRequest& req = it->second;
    TagOp& tag_op = req.op;
    DynInstPtr& inst = req.inst;
    if(req.isWrite) {
        DPRINTF(TagController, "tag controller write response\n");
        // TODO: potentially mark the instruction as complete
        // compare with store completion

    } else{
        DPRINTF(TagController, "tag controller query response\n");
        bool tag = pkt->getRaw<bool>();

        inst->completeTagQuery(tag_op.addr, tag);
        //iew->instToCommitIfExeced(inst);
        // TODO: handle fault and 
        // check whether the instruction execution is complete
    }
    
    ongoingRequests.erase(it);

    return true;
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

void
MemoryTagController::writeback() {
    while(!blockedRequests.empty() && tcachePort.canSend()) {
        auto req = blockedRequests.front();
        blockedRequests.pop();
        tcachePort.trySendPacket(req.first);

        assert(ongoingRequests.find(req.first->id) == ongoingRequests.end());
        ongoingRequests[req.first->id] = req.second;        
    }
    
    BaseTagController::writeback();
}


}
}
}

