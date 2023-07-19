#include "base/types.hh"
#include "base/flags.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "base/amo.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/node.hh"
#include "arch/riscvcapstone/insts/amo.hh"
#include "debug/NodeCmd.hh"

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
    req->setSize(sizeof(Node));
    PacketPtr pkt = Packet::createRead(req);
    //pkt->setSize(sizeof(Node)); // FIXME: do we need to specify the size here?
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
    req->setSize(sizeof(Node));
    PacketPtr pkt = Packet::createWrite(req);
    //pkt->setSize(sizeof(Node));
    pkt->allocate();
    memcpy(pkt->getPtr<void>(), &node, sizeof(Node));

    return pkt;
}

PacketPtr
NodeCommand::createStoreNode(const NodeID& node_id, const Node& node) {
    if(inst)
        return create_store_node(inst->requestorId(), node_id, node);
    return create_store_node(cpu->dataRequestorId(), node_id, node);
}

PacketPtr
NodeCommand::createLoadNode(const NodeID& node_id) {
    if(inst)
        return create_load_node(inst->requestorId(), node_id);
    return create_load_node(cpu->dataRequestorId(), node_id);
}

PacketPtr
NodeAllocate::transition() {
    DPRINTF(NodeCmd, "Node allocate transition for instruction %u (state = %u)\n",
            inst->seqNum, static_cast<unsigned int>(state)); 

    PacketPtr pkt = nullptr;

    NodeController& node_controller = inst->getNodeController();
    switch(state) {
        case NCAllocate_LOAD_PARENT:
            assert(toAllocate != NODE_ID_INVALID);
            // TODO: should attempt to reclaim nodes here instead
            DPRINTF(NodeCmd, "Allocated node ID = %u, parentID = %lu\n",
                    toAllocate, parentId);
            if(parentId == NODE_ID_INVALID) {
                DPRINTF(NodeCmd, "Allocated node becomes new root\n");

                // replace the root
                nextNodeId = node_controller.getRoot();
                parentDepth = 0;
                node_controller.setRoot(toAllocate);
                pkt = createLoadNode(toAllocate);

                state = NCAllocate_LOAD; // no need to access the root
            } else {
                pkt = createLoadNode(parentId);
            }
            break;
        case NCAllocate_STORE_PARENT:
            pkt = createStoreNode(parentId, savedNode);
            break;
        case NCAllocate_LOAD:
            pkt = createLoadNode(toAllocate);
            break;
        case NCAllocate_STORE:
            pkt = createStoreNode(toAllocate, savedNode);
            break;
        case NCAllocate_LOAD_RIGHT:
            pkt = createLoadNode(nextNodeId);
            break;
        case NCAllocate_STORE_RIGHT:
            pkt = createStoreNode(nextNodeId, savedNode);
            break;
        default:
            panic("Unrecognised state in node allocation!");
    }

    if(pkt) {
        status = AWAIT_CACHE;
    }
    return pkt;
}

void
NodeAllocate::handleResp(PacketPtr pkt) {
    assert(status == AWAIT_CACHE);
    switch(state) {
        case NCAllocate_LOAD_PARENT:
            savedNode = pkt->getRaw<Node>();
            parentDepth = savedNode.depth;
            nextNodeId = savedNode.next;
            savedNode.next = toAllocate;

            state = NCAllocate_STORE_PARENT;
            status = TO_RESUME;
            break;
        case NCAllocate_STORE_PARENT:
            state = NCAllocate_LOAD;
            status = TO_RESUME;
            break;
        case NCAllocate_LOAD:
            savedNode = pkt->getRaw<Node>();
            savedNode.prev = parentId;
            savedNode.depth = parentDepth + (asChild ? 1 : 0);
            savedNode.next = nextNodeId;
            savedNode.state = Node::VALID;
            savedNode.counter = 0; // TODO: perhaps 1
            
            state = NCAllocate_STORE;
            status = TO_RESUME;
            break;
        case NCAllocate_STORE:
            if(nextNodeId == NODE_ID_INVALID) {
                status = COMPLETED;
            } else {
                state = NCAllocate_LOAD_RIGHT;
                status = TO_RESUME;
            }
            break;
        case NCAllocate_LOAD_RIGHT:
            savedNode = pkt->getRaw<Node>();
            savedNode.prev = toAllocate;

            state = NCAllocate_STORE_RIGHT;
            status = TO_RESUME;
            break;
        case NCAllocate_STORE_RIGHT:
            status = COMPLETED;
            break;
        default:
            panic("Unrecognised state in node allocation!");
    }

    delete pkt;
}


PacketPtr
NodeQuery::transition() {
    DPRINTF(NodeCmd, "NodeQuery nodeId = %u\n",
            static_cast<unsigned int>(nodeId));
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

    DPRINTF(NodeCmd, "Query validityError for instruction %u = %u"
            " (node %u state = %u depth = %u prev = %u next = %u counter = %u)\n",
            inst->seqNum, validityError,
            static_cast<unsigned int>(nodeId),
            static_cast<unsigned int>(node.state),
            node.depth,
            static_cast<unsigned int>(node.prev),
            static_cast<unsigned int>(node.next),
            node.counter);

    delete pkt;
}

bool
NodeQuery::error() {
    return validityError;
}

PacketPtr
NodeQueryDbg::transition() {
    DPRINTF(NodeCmd, "NodeQuery nodeId = %u\n",
            static_cast<unsigned int>(nodeId));
    assert(status == NOT_STARTED);
    status = AWAIT_CACHE;
    return createLoadNode(nodeId);
}

void
NodeQueryDbg::handleResp(PacketPtr pkt) {
    status = COMPLETED;

    // check validity of the node
    Node node = pkt->getRaw<Node>();

    DPRINTFN("[sn:%u] Query for node = %u"
            " (state = %u, depth = %u, prev = %u, next = %u, RC = %u)\n",
            inst->seqNum,
            static_cast<unsigned int>(nodeId),
            static_cast<unsigned int>(node.state),
            node.depth,
            static_cast<unsigned int>(node.prev),
            static_cast<unsigned int>(node.next),
            node.counter);

    delete pkt;
}

bool
NodeQueryDbg::error() {
    return false;
}

PacketPtr
NodeRevoke::transition() {
    DPRINTF(NodeCmd, "NodeRevoke transition (state = %u)\n",
            static_cast<unsigned int>(state));
    PacketPtr pkt = nullptr;
    switch(state) {
        case NCRevoke_LOAD_ROOT:
            pkt = createLoadNode(nodeId);
            break;
        case NCRevoke_LOAD:
            pkt = createLoadNode(curNodeId);
            break;
        case NCRevoke_STORE:
        case NCRevoke_STORE_RIGHT:
            pkt = createStoreNode(curNodeId, savedNode);
            break;
        case NCRevoke_STORE_ROOT:
            rootNode.next = curNodeId;
            pkt = createStoreNode(nodeId, rootNode);
            break;
        default:
            panic("incorrect state for node revocation operation!");
    }
    if(pkt){
        status = AWAIT_CACHE;
    }
    return pkt;
}


void
NodeRevoke::handleResp(PacketPtr pkt) {
    assert(status == AWAIT_CACHE);
    NodeController& node_controller = inst->getNodeController();
    switch(state) {
        case NCRevoke_LOAD_ROOT:
            rootNode = pkt->getRaw<Node>();
            rootDepth = rootNode.depth;
            curNodeId = rootNode.next;

            if(curNodeId == NODE_ID_INVALID) { // subtree is empty
                // nothing needs to be done
                status = COMPLETED;
            } else{
                // don't mess with root
                state = NCRevoke_LOAD;
                status = TO_RESUME;
            }
            break;
        case NCRevoke_LOAD:
            savedNode = pkt->getRaw<Node>();
            if(savedNode.depth > rootDepth) {
                // still in the subtree
                savedNode.state = Node::INVALID;
                nextNodeId = savedNode.next;
                if(savedNode.counter == 0){
                    // immediately frees the node
                    node_controller.freeNode(savedNode, curNodeId);
                }
                state = NCRevoke_STORE;
                status = TO_RESUME;
            } else{
                // outside subtree
                // current node is the right node
                // need to update its prevNodeId
                savedNode.prev = nodeId;
                state = NCRevoke_STORE_RIGHT;
                status = TO_RESUME;
            }
            break;
        case NCRevoke_STORE:
            curNodeId = nextNodeId;
            if(curNodeId == NODE_ID_INVALID) {
                state = NCRevoke_STORE_ROOT;
                status = TO_RESUME;
            } else{
                state = NCRevoke_LOAD;
                status = TO_RESUME;
            }
            break;
        case NCRevoke_STORE_RIGHT:
            state = NCRevoke_STORE_ROOT;
            status = TO_RESUME;
            break;
        case NCRevoke_STORE_ROOT:
            status = COMPLETED;
            break;
    }
    delete pkt;
}

void
NodeRcUpdate::handleResp(PacketPtr pkt) {
    switch(state) {
        case NCRcUpdate_LOAD:
            savedNode = pkt->getRaw<Node>();
            savedNode.counter += delta;
            if(savedNode.counter == 0 && savedNode.state == 0) {
                // add node to free list
                if(inst)
                    inst->getNodeController().freeNode(savedNode, nodeId);
                else
                    cpu->nodeController.freeNode(savedNode, nodeId);
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
    delete pkt;
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

void
NodeRcUpdate::dump() {
    if(inst)
        DPRINTF(NodeCmd, "Inst sn = %u, command = %u, status = %u\n",
                inst->seqNum, getType(), status);
    else
        DPRINTF(NodeCmd, "Inst sn = %u, command = %u, status = %u\n",
                seqNum, getType(), status);
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
    assert(status == AWAIT_CACHE);
    assert(pkt && pkt->isResponse());
    bool completed = false;
    if(lockState == BEFORE_ACQUIRE) {
        if(*(pkt->getPtr<int>()) == 0) {
            lockState = ACQUIRED;
        }

        delete pkt;

        // otherwise, the lock was unavailable
    } else if(lockState == ACQUIRED) {
        if(rawCommand->status == COMPLETED) {
            // the lock has been released
            lockState = RELEASED;
            completed = true;
            delete pkt;
        } else {
            rawCommand->handleResp(pkt);
        }
    } else {
        delete pkt;
    }
    if(completed){
        status = COMPLETED;
    } else {
        status = TO_RESUME;
    }
}

PacketPtr
NodeDrop::transition() {
    DPRINTF(NodeCmd, "NodeDrop (nodeId = %u) transition (state = %u)\n",
            nodeId, static_cast<unsigned int>(state));
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

            status = TO_RESUME;
            state = NCDrop_STORE;
            break;
        case NCDrop_STORE:
            if(prevNodeId == NODE_ID_INVALID &&
                nextNodeId == NODE_ID_INVALID) {
                // the tree's only node is invalidated
                inst->getNodeController().setRoot(NODE_ID_INVALID);
                status = COMPLETED;
            } else if(prevNodeId == NODE_ID_INVALID) {
                inst->getNodeController().setRoot(nodeId);
                state = NCDrop_LOAD_RIGHT;
                status = TO_RESUME;
            } else {
                // if prev node exists
                state = NCDrop_LOAD_LEFT;
                status = TO_RESUME;
            }
            break;
        case NCDrop_LOAD_LEFT:
            savedNode = pkt->getRaw<Node>();
            savedNode.next = nextNodeId;

            state = NCDrop_STORE_LEFT;
            status = TO_RESUME;
            break;
        case NCDrop_STORE_LEFT:
            if(nextNodeId == NODE_ID_INVALID) {
                status = COMPLETED;
            } else {
                state = NCDrop_LOAD_RIGHT;
                status = TO_RESUME;
            }
            break;
        case NCDrop_LOAD_RIGHT:
            savedNode = pkt->getRaw<Node>();
            savedNode.prev = prevNodeId;

            state = NCDrop_STORE_RIGHT;
            status = TO_RESUME;
            break;
        case NCDrop_STORE_RIGHT:
            status = COMPLETED;
            break;
        default:
            panic("Unrecognised state in drop!");
    }
    delete pkt;
}

}
}
}
