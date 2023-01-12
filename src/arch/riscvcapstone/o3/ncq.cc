#include "arch/riscvcapstone/o3/ncq.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/cpu.hh"
#include "arch/riscvcapstone/o3/iew.hh"
#include "debug/NCQ.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


// TODO: we need to simulate the number of ports
NCQ::NCQ(CPU* cpu, IEW* iew, int queue_size, int thread_num) : 
    cpu(cpu), iew(iew),
    queueSize(queue_size),
    threadNum(thread_num),
    activeThreads(nullptr),
    ncachePort(this, cpu, 8) {
    
    threads.reserve(thread_num);
    for(int i = 0; i < thread_num; i ++) {
        threads.emplace_back(static_cast<ThreadID>(i), queue_size, cpu,
                this, iew);
    }
}

void 
NCQ::insertInstruction(const DynInstPtr& inst) {
    assert(inst->threadNumber >= 0 && inst->threadNumber < threadNum);
    threads[inst->threadNumber].insertInstruction(inst);
}

void
NCQ::tick() {
    ncachePort.tick();
    for(auto& t : threads)
        t.tick();
}

Fault
NCQ::pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd) {
    assert(inst->threadNumber >= 0 && inst->threadNumber < threadNum);
    DPRINTF(NCQ, "Push command to NCQ\n");
    return threads[inst->threadNumber].pushCommand(inst, cmd);
}

bool
NCQ::isFull(ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    return threads[thread_id].isFull();
}

Fault
NCQ::executeNodeOp(const DynInstPtr& inst) {
    DPRINTF(NCQ, "Executing node op insn %u\n", inst->seqNum);
    return inst->initiateNodeAcc();
}

void
NCQ::commitBefore(InstSeqNum seq_num, ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    DPRINTF(NCQ, "Committing all instructions before %u in NCQ on thread %u\n",
            seq_num, thread_id);
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

void
NCQ::cacheUnblocked() {
    DPRINTF(NCQ, "Node cache unblocked\n");
}


bool
NCQ::trySendPacket(PacketPtr pkt, ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    bool res = ncachePort.trySendPacket(pkt);

    assert(packetIssuerThreads.find(pkt->id) == packetIssuerThreads.end());
    packetIssuerThreads[pkt->id] = thread_id;

    return res;
}

bool
NCQ::handleCacheResp(PacketPtr pkt) {
    auto it = packetIssuerThreads.find(pkt->id);
    assert(it != packetIssuerThreads.end());
    ThreadID issuer_thread = it->second;
    assert(issuer_thread >= 0 && issuer_thread < threadNum);
    packetIssuerThreads.erase(it);

    // deliver to the NCQ unit of the issuer thread
    return threads[issuer_thread].handleCacheResp(pkt);
}

/** Ncache Port */
NCQ::NcachePort::NcachePort(NCQ* ncq, CPU* cpu, int size) :
    RequestPort(cpu->name() + ".ncache_port", cpu),
    ncq(ncq), cpu(cpu),
    portSize(size), portUsed(0),
    blockedPacket(nullptr), blocked(false) {
}

bool
NCQ::NcachePort::recvTimingResp(PacketPtr pkt) {
    return ncq->handleCacheResp(pkt); 
}

void
NCQ::NcachePort::recvReqRetry() {
    assert(blocked);
    blocked = false;
    ncq->cacheUnblocked();
    trySendPacket(blockedPacket);
}

bool
NCQ::NcachePort::trySendPacket(PacketPtr pkt) {
    assert(canSend());
    ++ portUsed;
    if(!sendTimingReq(pkt)) {
        blockedPacket = pkt;
        return false;
    }
    return true;
}

void
NCQ::NcachePort::tick() {
    portUsed = 0;
}

bool
NCQ::passedQuery(const DynInstPtr& inst) const {
    const ThreadID& thread_id = inst->threadNumber;
    assert(thread_id >= 0 && thread_id < threadNum);
    return threads[thread_id].passedQuery(inst);
}

void
NCQ::cleanupCommands() {
    for(auto& thread : threads) {
        thread.cleanupCommands();
    }
}


void 
NCQ::squash(const InstSeqNum& squashed_num, ThreadID thread_id) {
    assert(thread_id >= 0 && thread_id < threadNum);
    threads[thread_id].squash(squashed_num);
}

}
}
}

