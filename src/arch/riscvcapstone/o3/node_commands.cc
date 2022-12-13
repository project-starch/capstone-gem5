#include "base/types.hh"
#include "base/flags.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "base/amo.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/node.hh"
#include "arch/riscvcapstone/insts/amo.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

const Addr NODE_MEM_BASE_ADDR = 0x7d0000000ULL;

inline Addr node_addr(NodeID node_id) {
    return (Addr)(NODE_MEM_BASE_ADDR + ((Addr)node_id * (sizeof(Node))));
}

static PacketPtr
create_load_node(RequestorID requestor_id, const NodeID& node_id) {
    Addr addr = node_addr(node_id);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestor_id);
    req->setPaddr(addr);
    PacketPtr pkt = Packet::createRead(req);
    pkt->setSize(sizeof(Node)); // FIXME: do we need to specify the size here?
    pkt->allocate();

    return pkt;
}

static PacketPtr 
create_store_node(RequestorID requestor_id, const NodeID& node_id,
        const Node& node) {
    Addr addr = node_addr(node_id);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestor_id);
    req->setPaddr(addr);
    PacketPtr pkt = Packet::createWrite(req);
    pkt->setSize(sizeof(Node));
    pkt->allocate();
    memcpy(pkt->getPtr<void>(), &node, sizeof(Node));

    return pkt;
}

PacketPtr
NodeCommand::createStoreNode(const NodeID& node_id, const Node& node) {
    return create_store_node(inst->requestorId(), node_id, node);
}

PacketPtr
NodeCommand::createLoadNode(const NodeID& node_id) {
    return create_load_node(inst->requestorId(), node_id);
}

PacketPtr
NodeAllocate::transition() {
    return nullptr;
}

void
NodeAllocate::handleResp(PacketPtr pkt) {
}




PacketPtr
NodeQuery::transition() {
    assert(status == NOT_STARTED);
    status = AWAIT_CACHE;
    return createLoadNode(nodeId);
}

void
NodeQuery::handleResp(PacketPtr pkt) {
    status = COMPLETED;

    // check validity of the node
    Node node = pkt->getRaw<Node>();
    validityError = !node.isValid();
}

bool
NodeQuery::error() {
    return validityError;
}

void
NodeRevoke::handleResp(PacketPtr pkt) {
#if(0)
    assert(status == AWAIT_CACHE);
    switch(state) {
        case NCRevoke_LOAD_ROOT:
            savedNode = *(pkt->getPtr<Node>());
            //savedNode = pkt->getRaw<Node>();
            rootDepth = savedNode.depth;
            curNodeId = savedNode.next;
            prevNodeId = savedNode.prev;
            savedNode.state = 0; // invalidate
            if(savedNode.counter == 0){
                // the node can be immediately freed
                //controller.freeNode(savedNode, nodeId);
            }

            state = NCRevoke_STORE;
            break;
        case NCRevoke_LOAD:
            savedNode = pkt->getRaw<Node>();
            if(savedNode.depth > rootDepth) {
                // still in the subtree
                savedNode.state = 0;
                old_node_id = curNodeId;
                curNodeId = savedNode.next;
                if(savedNode.counter == 0){
                    // immediately frees the node
                    //controller.freeNode(savedNode, old_node_id);
                }
                //pkt = create_store_node(inst->requestorId(), old_node_id, savedNode);

                state = NCRevoke_STORE;
            } else{
                // outside subtree
                // current node is the right node
                // need to update its prevNodeId
                savedNode.prev = prevNodeId;
                //pkt = create_store_node(inst->requestorId(), curNodeId, savedNode);
                state = NCRevoke_STORE_RIGHT;
            }
            break;
        case NCRevoke_LOAD_LEFT:
            savedNode = pkt->getRaw<Node>();
            savedNode.next = curNodeId;
            state = NCRevoke_STORE_LEFT;
            break;
        case NCRevoke_STORE_LEFT:
            break;
        case NCRevoke_STORE:
            if(curNodeId == NODE_ID_INVALID) {
                if(prevNodeId == NODE_ID_INVALID) {
                    // the tree is empty
                    //controller.tree_root = NODE_ID_INVALID;
                    //current_pkt->makeResponse();
                    //current_pkt->deleteData();
                }
                // need to change prev->next
                //pkt = create_load_node(inst->requestorId(), prevNodeId);
                state = NCRevoke_LOAD_LEFT;
            } else{
                //pkt = create_load_node(inst->requestorId(), curNodeId);
                state = NCRevoke_LOAD;
            }
            break;
    }
#endif
}

PacketPtr
NodeRevoke::transition() {
#if(0)
    NodeID old_node_id;
    PacketPtr pkt = nullptr;
    switch(state) {
        case NCRevoke_LOAD_ROOT:
            pkt = create_store_node(inst->requestorId(), nodeId, node);
            state = NCRevoke_STORE;

            break;
        case NCRevoke_LOAD:
            node = pkt->getRaw<Node>();
            break;
        case NCRevoke_STORE_RIGHT:
            if(prevNodeId == NODE_ID_INVALID) {
                //controller.tree_root = curNodeId;
                //current_pkt->makeResponse();
                //current_pkt->deleteData();
            } else{
                pkt = create_load_node(inst->requestorId(), prevNodeId);
                state = NCRevoke_LOAD_LEFT;
            }
            break;
        case NCRevoke_LOAD_LEFT:
            pkt = create_store_node(inst->requestorId(), prevNodeId, node);
            break;
        case NCRevoke_STORE_LEFT:
            //current_pkt->makeResponse();
            //current_pkt->deleteData();
            break;
        case NCRevoke_STORE:
            break;
        default:
            panic("incorrect state for node revocation operation!");
    }
    if(pkt){
        status = AWAIT_CACHE;
    }
    return pkt;
#else
    return nullptr;
#endif
}

void
NodeRcUpdate::handleResp(PacketPtr pkt) {
    switch(state) {
        case NCRcUpdate_LOAD:
            savedNode = pkt->getRaw<Node>();
            savedNode.counter += delta;
            if(savedNode.counter == 0 && savedNode.state == 0) {
                // add node to free list
                inst->getNodeController().freeNode(savedNode, nodeId);
            }

            // note that we do not need to do anything 
            // with prev and next because they are invalid notes
            state = NCRcUpdate_STORE;
            status = TO_RESUME;
            break;
        case NCRcUpdate_STORE:
            status = COMPLETED;
            break;
        default:
            panic("unrecognised state in RcUpdate!");
    }
}


// when rc reaches 0
// if the node if invalid: add the node to the free list
// if the node is valid: no need to do anything
PacketPtr
NodeRcUpdate::transition() {
    PacketPtr pkt = nullptr;
    switch(state) {
        case NCRcUpdate_LOAD:
            pkt = createLoadNode(nodeId);
            break;
        case NCRcUpdate_STORE:
            pkt = createStoreNode(nodeId, savedNode);
            break;
        default:
            panic("unrecognised state in RcUpdate!");
    }
    if(pkt) {
        status = AWAIT_CACHE;
    }
    return pkt;
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
    req->setPaddr(NODE_MEM_BASE_ADDR);

    PacketPtr pkt = Packet::createWrite(req);

    // pkt->setSize(sizeof(int));
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
    req->setPaddr(NODE_MEM_BASE_ADDR);

    PacketPtr pkt = Packet::createWrite(req);

    //pkt->setSize(sizeof(int));
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

PacketPtr
NodeDrop::transition() {
    PacketPtr pkt = nullptr;
    switch(state) {
        case NCDrop_LOAD:
            pkt = createLoadNode(nodeId); // load the node to drop
            break;
        case NCDrop_STORE:
            pkt = createStoreNode(nodeId, savedNode);
            break;
        case NCDrop_LOAD_LEFT:
            pkt = createLoadNode(prevNodeId);
            break;
        case NCDrop_STORE_LEFT:
            pkt = createStoreNode(prevNodeId, savedNode);
            break;
        case NCDrop_LOAD_RIGHT:
            pkt = createLoadNode(nextNodeId);
            break;
        case NCDrop_STORE_RIGHT:
            pkt = createStoreNode(nextNodeId, savedNode);
            break;
        default:
            panic("Unrecognised state in drop!");
    }
    if(pkt) {
        status = AWAIT_CACHE;
    }
    return pkt;
}

void
NodeDrop::handleResp(PacketPtr pkt) {
    switch(state) {
        case NCDrop_LOAD:
            savedNode = pkt->getRaw<Node>();
            assert(savedNode.isValid()); // TODO: actually need to handle this case
            prevNodeId = savedNode.prev;
            nextNodeId = savedNode.next;
            savedNode.invalidate();

            state = NCDrop_STORE;
            break;
        case NCDrop_STORE:
            if(prevNodeId == NODE_ID_INVALID &&
                nextNodeId == NODE_ID_INVALID) {
                inst->getNodeController().setRoot(nodeId);
                status = COMPLETED;
            } else if(prevNodeId == NODE_ID_INVALID) {
                inst->getNodeController().setRoot(nodeId);
                state = NCDrop_LOAD_RIGHT;
            } else {
                // if prev node exists
                state = NCDrop_LOAD_LEFT;
            }
            break;
        case NCDrop_LOAD_LEFT:
            savedNode = pkt->getRaw<Node>();
            savedNode.next = nextNodeId;

            state = NCDrop_STORE_LEFT;
            break;
        case NCDrop_STORE_LEFT:
            if(nextNodeId == NODE_ID_INVALID) {
                status = COMPLETED;
            } else {
                state = NCDrop_LOAD_RIGHT;
            }
            break;
        case NCDrop_LOAD_RIGHT:
            savedNode = pkt->getRaw<Node>();
            savedNode.prev = prevNodeId;

            state = NCDrop_STORE_RIGHT;
            break;
        case NCDrop_STORE_RIGHT:
            status = COMPLETED;
            break;
        default:
            panic("Unrecognised state in drop!");
    }
}

}
}
}
