#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "node_commands.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


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
PacketPtr
LockedNodeCommand::transition() {
    assert(lockState != RELEASED && status != COMPLETED &&
            status != AWAIT_CACHE && rawCommand->status != AWAIT_CACHE);

    PacketPtr pkt = nullptr;

    if(lockState == BEFORE_ACQUIRE) {
        assert(rawCommand->status == NOT_STARTED);
        // TODO: construct a packet for atomic rw node 0
    } else if(lockState == ACQUIRED) {
        assert(status != NOT_STARTED);
        if(rawCommand->status == COMPLETED) {
            // TODO: release the lock
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
    bool completed = false;
    if(lockState == BEFORE_ACQUIRE) {
        // TODO: check if the lock acquire is successful
        bool lock_acquired = false;
        if(lock_acquired) {
            lockState = ACQUIRED;
        }
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
}

}
}
}
