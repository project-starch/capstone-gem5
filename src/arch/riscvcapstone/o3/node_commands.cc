#include "base/types.hh"
#include "base/flags.hh"
#include "base/amo.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/insts/amo.hh"
#include "node_commands.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

const Addr NODE_MEM_BASE_ADDR = 0x7d0000000ULL;

PacketPtr
NodeAllocate::transition() {
    return nullptr;
}

void
NodeAllocate::handleResp(PacketPtr pkt) {
}


PacketPtr
NodeRevoke::transition() {
    return nullptr;
}

void
NodeRevoke::handleResp(PacketPtr pkt) {
}


PacketPtr
NodeRcUpdate::transition() {
    return nullptr;
}

void
NodeRcUpdate::handleResp(PacketPtr pkt) {
}


PacketPtr
NodeQuery::transition() {
    return nullptr;
}

void
NodeQuery::handleResp(PacketPtr pkt) {
}

/** 
 *
 * Locked command
 * */
// TODO: actually fence is necessary
PacketPtr
LockedNodeCommand::createAcquirePacket() {
    RequestPtr req = std::make_shared<Request>(
        NODE_MEM_BASE_ADDR,
        sizeof(int),
        Flags<Request::FlagsType>(), // no flags needed
        inst->requestorId(),
        inst->pcState().instAddr(),
        inst->contextId(),
        std::make_unique<AtomicGenericOp<uint32_t> >(
            1, [](uint32_t* a, uint32_t b) { *a = b; }
        )
    );

    PacketPtr pkt = Packet::createWrite(req);

    pkt->setSize(sizeof(int));
    pkt->allocate();
    *(pkt->getPtr<int>()) = 1;

    return pkt;
}

PacketPtr
LockedNodeCommand::createReleasePacket() {
    RequestPtr req = std::make_shared<Request>(
        NODE_MEM_BASE_ADDR,
        4,
        Flags<Request::FlagsType>(), // no flags needed
        inst->requestorId(),
        inst->pcState().instAddr(),
        inst->contextId()
    );

    PacketPtr pkt = Packet::createWrite(req);

    pkt->setSize(sizeof(int));
    pkt->allocate();
    *(pkt->getPtr<int>()) = 0;

    return pkt;
}

PacketPtr
LockedNodeCommand::transition() {
    assert(lockState != RELEASED && status != COMPLETED &&
            status != AWAIT_CACHE && rawCommand->status != AWAIT_CACHE);

    PacketPtr pkt = nullptr;

    if(lockState == BEFORE_ACQUIRE) {
        assert(rawCommand->status == NOT_STARTED);
        pkt = createAcquirePacket();
        // construct a packet for atomic rw node 0
    } else if(lockState == ACQUIRED) {
        assert(status != NOT_STARTED);
        if(rawCommand->status == COMPLETED) {
            pkt = createReleasePacket();
        } else {
            pkt = rawCommand->transition();
        }
    }
    
    if(pkt) {
        status = AWAIT_CACHE;
    }

    return pkt;
}

void
LockedNodeCommand::handleResp(PacketPtr pkt) {
    assert(status == AWAIT_CACHE && rawCommand->status == AWAIT_CACHE);
    assert(pkt && pkt->isResponse());
    bool completed = false;
    if(lockState == BEFORE_ACQUIRE) {
        if(*(pkt->getPtr<int>()) == 0) {
            lockState = ACQUIRED;
        }

        // otherwise, the lock was unavailable
    } else if(lockState == ACQUIRED) {
        if(rawCommand->status == COMPLETED) {
            // the lock has been released
            lockState = RELEASED;
            completed = true;
        } else {
            rawCommand->handleResp(pkt);
        }
    }
    if(completed){
        status = COMPLETED;
    } else {
        status = TO_RESUME;
    }

    delete pkt;
}

}
}
}
