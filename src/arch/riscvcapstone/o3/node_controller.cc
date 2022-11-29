#include<stdexcept>
#include<cstring>
#include "mem/packet_access.hh"
#include "arch/riscvcapstone/o3/node_controller.hh"
#include "base/trace.hh"
#include "debug/CapstoneNCache.hh"
#include "debug/CapstoneCapTrack.hh"
#include "debug/CapstoneNodeOps.hh"
#include "debug/CapstoneNodeOpsAtomic.hh"
#include "debug/NodeController.hh"


/*
 *
 * Note: current implementation is not very performant.
 * The whole revocation node subsystem will be blocked for every
 * request.
 *
 * Might consider optimising this in the future.
 * */


namespace gem5::RiscvcapstoneISA::o3 {

NodeController::NodeController(const CapstoneO3NodeControllerParams& p) :
    ClockedObject(p),
    stats(this),
    currentCmd(NULL),
    mem_side(this),
    system(p.system),
    freeNodeInited(0),
    free_head(NODE_ID_INVALID),
    tree_root(NODE_ID_INVALID) {
    DPRINTF(CapstoneNCache, "Size of node = %u\n", sizeof(Node));
}

bool
NodeController::MemSidePort::recvTimingResp(PacketPtr pkt) {
    owner->handleResp(pkt); // always succeed
    // no need to send retry response because responses are
    // always successfully handled
    return true;
}


void
NodeController::MemSidePort::recvReqRetry() {
    assert(retryPkt != NULL);
    PacketPtr pkt = retryPkt;
    retryPkt = NULL;
    trySendReq(pkt);
}

void
NodeController::sendPacketToMem(PacketPtr pkt, bool atomic) {
    if(atomic) {
        mem_side.sendAtomic(pkt);
    } else {
        mem_side.trySendReq(pkt);
    }
}

PacketPtr
NodeController::sendLoad(NodeControllerCommandPtr cmd, NodeID node_id, bool atomic) {
    switch(cmd->getType()) {
        case NodeControllerCommand::Type::ALLOCATE:
            ++ stats.allocatePacketLoadCount;
            break;
        case NodeControllerCommand::Type::REVOKE:
            ++ stats.revokePacketLoadCount;
            break;
        case NodeControllerCommand::Type::RC_UPDATE:
            ++ stats.rcUpdatePacketLoadCount;
            break;
        case NodeControllerCommand::Type::QUERY:
            ++ stats.queryPacketLoadCount;
            break;
        default:;
    }

    Addr addr = nodeId2Addr(node_id);
    DPRINTF(CapstoneNodeOps, "send load %lx\n", addr);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestorId);
    req->setPaddr(addr);
    PacketPtr pkt = Packet::createRead(req);
    pkt->setSize(sizeof(Node)); // FIXME: do we need to specify the size here?
    pkt->allocate();

    sendPacketToMem(pkt, atomic);
    return pkt;
}

PacketPtr
NodeController::sendStore(NodeControllerCommandPtr cmd, NodeID node_id, const Node& node, 
        bool atomic) {
    switch(cmd->getType()) {
        case NodeControllerCommand::Type::ALLOCATE:
            ++ stats.allocatePacketStoreCount;
            break;
        case NodeControllerCommand::Type::REVOKE:
            ++ stats.revokePacketStoreCount;
            break;
        case NodeControllerCommand::Type::RC_UPDATE:
            ++ stats.rcUpdatePacketStoreCount;
            break;
        case NodeControllerCommand::Type::QUERY:
            ++ stats.queryPacketStoreCount;
            break;
        default:;
    }

    Addr addr = nodeId2Addr(node_id);
    DPRINTF(CapstoneNodeOps, "send store %lx\n", addr);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestorId);
    req->setPaddr(addr);
    PacketPtr pkt = Packet::createWrite(req);
    pkt->setSize(sizeof(Node));
    pkt->allocate();
    memcpy(pkt->getPtr<void>(), &node, sizeof(Node));

    sendPacketToMem(pkt, atomic);
    return pkt;
}

bool
NodeControllerQuery::transit(NodeController& controller, PacketPtr pkt) {
    data = new Node;
    memcpy((void*)data, pkt->getPtr<void>(), sizeof(Node));

    return true;
}

bool
NodeControllerAllocate::transit(NodeController& controller, PacketPtr pkt) {
    Node node;
    switch(state) {
        case NCAllocate_LOAD_PARENT:
            node = pkt->getRaw<Node>();
            parentDepth = node.depth;
            nextNodeId = node.next;
            node.next = toAllocate;

            state = NCAllocate_STORE_PARENT;
            controller.sendStore(this, parentId, node);
            return false;
        case NCAllocate_STORE_PARENT:
            if(nextNodeId == NODE_ID_INVALID) {
                state = NCAllocate_LOAD;
                controller.sendLoad(this, toAllocate);
            } else{
                state = NCAllocate_LOAD_RIGHT;
                controller.sendLoad(this, nextNodeId);
            }
            return false;
        case NCAllocate_LOAD_RIGHT:
            node = pkt->getRaw<Node>();
            node.prev = toAllocate;
            state = NCAllocate_STORE_RIGHT;
            controller.sendStore(this, nextNodeId, node);
            return false;
        case NCAllocate_STORE_RIGHT:
            state = NCAllocate_LOAD;
            controller.sendLoad(this, toAllocate);
            return false;
        case NCAllocate_LOAD:
            node = pkt->getRaw<Node>(); // free node
            nextFreeNodeId = node.next;
            node.prev = parentId;
            node.next = nextNodeId;
            if(parentId == NODE_ID_INVALID) {
                controller.tree_root = toAllocate; // may not be necessary?
            }
            node.state = 1;
            node.counter = 1;
            node.depth = parentDepth + 1;
            controller.sendStore(this, toAllocate, node);

            state = NCAllocate_STORE;
            
            return false;
        case NCAllocate_STORE:
            if(fromFreeList) {
                controller.free_head = nextFreeNodeId;
            } else{
                ++ controller.freeNodeInited;
            }
            data = toAllocate;
            // TODO: consider returning status code
            return true;
        default:
            panic("incorrect state for node allocation operation!");
    }
    
}

Tick
NodeControllerQuery::handleAtomic(NodeController& controller) {
    this->data = new Node;
    controller.atomicLoadNode(this, nodeId, this->data);
    
    return 0;
}

NodeControllerQuery::~NodeControllerQuery() {
    if(this->data != NULL) {
        delete this->data;
    }
}


Tick
NodeControllerRevoke::handleAtomic(NodeController& controller) {
    Node node;
    controller.atomicLoadNode(this, nodeId, &node);
    rootDepth = node.depth;
    curNodeId = node.next;
    prevNodeId = node.prev;
    node.state = 0; // invalidate the node
    if(node.counter == 0) {
        controller.freeNode(node, nodeId);
    }
    controller.atomicStoreNode(this, nodeId, &node);
    while(curNodeId != NODE_ID_INVALID) {
        controller.atomicLoadNode(this, curNodeId, &node);
        if(node.depth > rootDepth) {
            node.state = 0;
            NodeID next = node.next;
            if(node.counter == 0){
                controller.freeNode(node, curNodeId);
            }
            controller.atomicStoreNode(this, curNodeId, &node);
            curNodeId = next;
        } else{
            node.prev = prevNodeId; 
            controller.atomicStoreNode(this, curNodeId, &node);
            break;
        }
    }
    if(prevNodeId == NODE_ID_INVALID) {
        controller.tree_root = curNodeId;
    } else {
        controller.atomicLoadNode(this, prevNodeId, &node);
        node.next = curNodeId;
        controller.atomicStoreNode(this, prevNodeId, &node);
    }

    return 0;
}


Tick
NodeControllerRcUpdate::handleAtomic(NodeController& controller) {
    panic_if(delta == 0, "node controller does not allow rc updating with delta = 0 (atomic)");

    Node node;
    controller.atomicLoadNode(this, nodeId, &node);
    DPRINTF(CapstoneNodeOpsAtomic, "rc update %llu %d %d %d\n", nodeId, node.counter, delta, node.state);
    node.counter += delta;
    if(node.counter == 0 && node.state == 0) { // now I can free this node
        controller.freeNode(node, nodeId);
    }
    controller.atomicStoreNode(this, nodeId, &node);
    
    return 0;
}

Tick
NodeControllerAllocate::handleAtomic(NodeController& controller) {
    Node node;

    if(controller.free_head == NODE_ID_INVALID) {
        panic_if(controller.freeNodeInited >= CAPSTONE_NODE_N, "no free node remaining (atomic).");
        toAllocate = (NodeID)controller.freeNodeInited;
        fromFreeList = false;
    } else{
        toAllocate = controller.free_head;
        fromFreeList = true;
    }

    if(parentId == NODE_ID_INVALID) {
        nextNodeId = controller.tree_root;
        parentDepth = 0;
    } else{
        controller.atomicLoadNode(this, parentId, &node);
        nextNodeId = node.next;
        parentDepth = node.depth;
        node.next = toAllocate;
        controller.atomicStoreNode(this, parentId, &node);
    }

    if(nextNodeId != NODE_ID_INVALID) {
        controller.atomicLoadNode(this, nextNodeId, &node);
        node.prev = toAllocate;
        controller.atomicStoreNode(this, nextNodeId, &node);
    }

    controller.atomicLoadNode(this, toAllocate, &node);
    nextFreeNodeId = node.next;
    node.prev = parentId;
    node.next = nextNodeId;
    node.depth = parentDepth + 1;
    node.counter = 1;
    node.state = 1;
    if(parentId == NODE_ID_INVALID) {
        controller.tree_root = toAllocate;
    }
    controller.atomicStoreNode(this, toAllocate, &node);
    
    if(fromFreeList) {
        controller.free_head = nextFreeNodeId;
    } else{
        ++ controller.freeNodeInited;
    }

    this->data = toAllocate;

    return 0;
}

bool
NodeControllerRevoke::transit(NodeController& controller, PacketPtr pkt) {
    Node node;
    NodeID old_node_id;
    switch(state) {
        case NCRevoke_LOAD_ROOT:
            node = pkt->getRaw<Node>();
            rootDepth = node.depth;
            curNodeId = node.next;
            prevNodeId = node.prev;
            node.state = 0; // invalidate
            if(node.counter == 0){
                // the node can be immediately freed
                controller.freeNode(node, nodeId);
            }
            controller.sendStore(this, nodeId, node);

            state = NCRevoke_STORE;

            return false;
        case NCRevoke_LOAD:
            node = pkt->getRaw<Node>();
            if(node.depth > rootDepth) {
                // still in the subtree
                node.state = 0;
                old_node_id = curNodeId;
                curNodeId = node.next;
                if(node.counter == 0){
                    // immediately frees the node
                    controller.freeNode(node, old_node_id);
                }
                controller.sendStore(this, old_node_id, node);

                state = NCRevoke_STORE;
            } else{
                // outside subtree
                // current node is the right node
                // need to update its prevNodeId
                node.prev = prevNodeId;
                controller.sendStore(this, curNodeId, node);
                state = NCRevoke_STORE_RIGHT;
            }
            return false;
        case NCRevoke_STORE_RIGHT:
            if(prevNodeId == NODE_ID_INVALID) {
                controller.tree_root = curNodeId;
                return true;
            }
            controller.sendLoad(this, prevNodeId);
            state = NCRevoke_LOAD_LEFT;
            return false;
        case NCRevoke_LOAD_LEFT:
            node = pkt->getRaw<Node>();
            node.next = curNodeId;
            controller.sendStore(this, prevNodeId, node);
            state = NCRevoke_STORE_LEFT;
            return false;
        case NCRevoke_STORE_LEFT:
            return true;
        case NCRevoke_STORE:
            if(curNodeId == NODE_ID_INVALID) {
                if(prevNodeId == NODE_ID_INVALID) {
                    // the tree is empty
                    controller.tree_root = NODE_ID_INVALID;
                    return true;
                }
                // need to change prev->next
                controller.sendLoad(this, prevNodeId);
                state = NCRevoke_LOAD_LEFT;
            } else{
                controller.sendLoad(this, curNodeId);
                state = NCRevoke_LOAD;
            }
            return false;
        default:
            panic("incorrect state for node revocation operation!");
    }
}

// when rc reaches 0
// if the node if invalid: add the node to the free list
// if the node is valid: no need to do anything
bool
NodeControllerRcUpdate::transit(NodeController& controller, PacketPtr pkt) {
    Node node;
    switch(state) {
        case NCRcUpdate_LOAD:
            node = pkt->getRaw<Node>();
            node.counter += delta;
            if(node.counter == 0 && node.state == 0) {
                // add node to free list
                controller.freeNode(node, nodeId);
                controller.sendStore(this, nodeId, node);

                // note that we do not need to do anything 
                // with prev and next because they are invalid notes

                state = NCRcUpdate_STORE;
            } else{
                controller.sendStore(this, nodeId, node);

                state = NCRcUpdate_STORE;
            }

            return false;
        case NCRcUpdate_STORE:
            return true;
        default:
            panic("incorrect state for ref count update operation!");
    }
}

void
NodeController::handleResp(PacketPtr pkt) {
    assert(currentCmd != NULL);

    bool finish = currentCmd->transit(*this, pkt);

    delete pkt;

    if(finish) {
        currentCmd = NULL;

        //delete cmd;
        // FIXME: do something instead
        //cpu_side.trySendResp(pkt);
    }
}

Port& NodeController::getPort(const std::string& name, PortID idx) {
    if(name == "mem_side")
        return mem_side;
    return ClockedObject::getPort(name, idx);
}

NodeController::MemSidePort::MemSidePort(NodeController* owner) :
    RequestPort(owner->name() + ".mem_side", owner),
    owner(owner),
    retryPkt(NULL)
{

}

Addr
NodeController::nodeId2Addr(NodeID node_id) {
    return (Addr)(CAPSTONE_NODE_BASE_ADDR + ((Addr)node_id * (sizeof(Node))));
}

void
NodeController::MemSidePort::trySendReq(PacketPtr pkt) {
    assert(retryPkt == NULL);
    assert(pkt->isRead() || pkt->isWrite());
    if(!sendTimingReq(pkt)) {
        retryPkt = pkt;
    }
}

void
NodeController::init() {
    ClockedObject::init();

    requestorId = system->getRequestorId(this);
}

void
NodeControllerQuery::setup(NodeController& controller) {
    DPRINTF(CapstoneNCache, "Read from node cache\n");

    controller.sendLoad(this, nodeId);
}


void
NodeControllerAllocate::setup(NodeController& controller) {
    if(controller.free_head == NODE_ID_INVALID) {
        panic_if(controller.freeNodeInited >= CAPSTONE_NODE_N, "no free node remaining.");
        toAllocate = (NodeID)controller.freeNodeInited;
        fromFreeList = false;
    } else{
        toAllocate = controller.free_head;
        fromFreeList = true;
    }
    // TODO: need to handle the case when there are no free nodes
    if(parentId == NODE_ID_INVALID) {
        // skip setting parent
        // same as in NCAllocate_STORE_PARENT
        nextNodeId = controller.tree_root;
        parentDepth = 0;
        if(nextNodeId == NODE_ID_INVALID) {
            state = NCAllocate_LOAD;
            controller.sendLoad(this, toAllocate);
        } else {
            state = NCAllocate_LOAD_RIGHT;
            controller.sendLoad(this, nextNodeId);
        }
    } else{
        // load parent first so we know the depth and the next node
        state = NCAllocate_LOAD_PARENT;
        controller.sendLoad(this, parentId);
    }
}

void
NodeControllerRcUpdate::setup(NodeController& controller) {
    DPRINTF(CapstoneNodeOps, "rcupdate: %u %d\n", nodeId, delta);
    assert(nodeId != NODE_ID_INVALID);
    assert(delta != 0);
    state = NCRcUpdate_LOAD;
    controller.sendLoad(this, nodeId);
}

void
NodeControllerRevoke::setup(NodeController& controller) {
    assert(nodeId != NODE_ID_INVALID);
    state = NCRevoke_LOAD_ROOT;
    controller.sendLoad(this, nodeId);
}


bool
NodeController::executeTiming(NodeControllerCommandPtr cmd) {
    if(currentCmd != NULL) {
        return false;
    }
    ++ stats.timingReqCount;
    
    panic_if(cmd == NULL, "node controller received invalid command");

    handleCommon(cmd);

    cmd->setup(*this);

    currentCmd = cmd;

    return true;
}


void
NodeController::handleCommon(NodeControllerCommandPtr cmd) {
    switch(cmd->getType()) {
        case NodeControllerCommand::Type::ALLOCATE:
            ++ stats.allocateCount;
            break;
        case NodeControllerCommand::Type::REVOKE:
            ++ stats.revokeCount;
            break;
        case NodeControllerCommand::Type::RC_UPDATE:
            ++ stats.rcUpdateCount;
            break;
        case NodeControllerCommand::Type::QUERY:
            ++ stats.queryCount;
            break;
        default:;
    }
}

Tick
NodeController::executeAtomic(NodeControllerCommandPtr cmd) {
    panic_if(cmd == NULL, "node controller received invalid command (atomic)");
    
    ++ stats.atomicReqCount;

    handleCommon(cmd);

    return cmd->handleAtomic(*this);

}

void
NodeController::allocObject(const SimpleAddrRange& obj) {
    //objectRanges.insert(obj);
}

void
NodeController::freeObject(Addr addr) {
}

void
NodeController::regStats() {
    ClockedObject::regStats();
}

void
NodeController::addCapTrack(const CapLoc& loc, NodeID node_id) {
    DPRINTF(CapstoneCapTrack, "cap track added with node %u, %s\n", node_id,
            loc.toString().c_str());
    capTrackMap[loc] = node_id;
}

NodeID
NodeController::queryCapTrack(const CapLoc& loc) {
    //DPRINTF(CapstoneCapTrack, "cap track queried\n");
    try{
        return capTrackMap.at(loc);
    } catch(const std::out_of_range& e) {
        return NODE_ID_INVALID;
    }
}

void
NodeController::removeCapTrack(const CapLoc& loc) {
    DPRINTF(CapstoneCapTrack, "cap track removed %s\n",
            loc.toString().c_str());
    capTrackMap.erase(loc);
}

void
NodeController::freeNode(Node& node, NodeID node_id) {
    DPRINTF(CapstoneNodeOps, "free node with id %lu\n", node_id);
    node.next = free_head;
    free_head = node_id;
}

NodeControllerCommand::Type NodeControllerAllocate::getType() const {
    return Type::ALLOCATE;
}

NodeControllerCommand::Type NodeControllerQuery::getType() const {
    return Type::QUERY;
}

NodeControllerCommand::Type NodeControllerRcUpdate::getType() const {
    return Type::RC_UPDATE;
}

NodeControllerCommand::Type NodeControllerRevoke::getType() const {
    return Type::REVOKE;
}

bool NodeControllerAllocate::readOnly() const {
    return false;
}

bool NodeControllerQuery::readOnly() const {
    return true;
}

bool NodeControllerRcUpdate::readOnly() const {
    return false;
}

bool NodeControllerRevoke::readOnly() const {
    return false;
}

void
NodeController::commitStores(InstSeqNum& ins_seq) {
    // brute force
    //for(NodeControllerCommandPtr cmd : storeBuffer) {
        //if(cmd->insn != nullptr && cmd->insn->seqNum <= ins_seq) {
            //// if older than the youngest committed instruction
            //// mark as can writeback
            //DPRINTF(NodeController, "commit store for %llu", cmd->insn->seqNum);
            //cmd->canWB = true;
        //}
    //}
}

void
NodeController::writebackStores() {
    //for(NodeControllerCommandPtr cmd : storeBuffer) {
        //if(cmd->canWB) {
            //// then just writeback
            //// send it to cache
        //}
    //}
}

bool
NodeController::tryInsert(DynInstPtr inst) {
    return true;
}

} // end of namespace gem5::RiscvcapstoneISA

