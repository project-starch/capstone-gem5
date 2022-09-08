#include<cstring>
#include "mem/packet_access.hh"
#include "arch/riscvcapstone/node_controller.hh"
#include "base/trace.hh"
#include "debug/CapstoneNCache.hh"


/*
 *
 * Note: current implementation is not very performant.
 * The whole revocation node subsystem will be blocked for every
 * request.
 *
 * Might consider optimising this in the future.
 * */

#define CAPSTONE_NODE_BASE_ADDR 0x100000000000ULL


namespace gem5::RiscvcapstoneISA {

NodeController::NodeController(const NodeControllerParams& p) :
    ClockedObject(p),
    currentPkt(NULL),
    mem_side(this),
    cpu_side(this),
    system(p.system),
    free_head(NODE_ID_INVALID),
    tree_root(NODE_ID_INVALID) {
    DPRINTF(CapstoneNCache, "Size of node = %u\n", sizeof(Node));
}

NodeController::CPUSidePort::CPUSidePort(NodeController* owner) :
    ResponsePort(owner->name() + ".cpu_side", owner),
    owner(owner),
    retryPkt(NULL),
    toRetryReq(false) {
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
NodeController::sendLoad(NodeID node_id) {
    Addr addr = nodeID2Addr(node_id);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestorId);
    req->setPaddr(addr);
    PacketPtr pkt = Packet::createRead(req);
    pkt->setSize(sizeof(Node)); // FIXME: do we need to specify the size here?
    pkt->allocate();

    mem_side.trySendReq(pkt);
}

void
NodeController::sendStore(NodeID node_id, const Node& node) {
    Addr addr = nodeID2Addr(node_id);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestorId);
    req->setPaddr(addr);
    PacketPtr pkt = Packet::createWrite(req);
    pkt->setSize(sizeof(Node));
    pkt->allocate();
    memcpy(pkt->getPtr<void>(), &node, sizeof(Node));

    mem_side.trySendReq(pkt);
}

bool
NodeControllerQuery::transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) {
    current_pkt->makeResponse();
    assert(current_pkt->getPtr<NodeControllerCommand>() != NULL);
    current_pkt->deleteData();
    current_pkt->setSize(sizeof(Node));
    current_pkt->allocate(); // TODO: let the cpu allocate
    memcpy(current_pkt->getPtr<void>(), pkt->getPtr<void>(), sizeof(Node));

    return true;
}

bool
NodeControllerAllocate::transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) {
    Node node;
    switch(state) {
        case NCAllocate_LOAD:
            node = pkt->getRaw<Node>(); // free node
            nextNodeId = node.next;
            // TODO: we need to mount the node at a location in the tree
            node.prev = node.next = NODE_ID_INVALID;
            node.state = 1;
            node.counter = 1;
            node.depth = 0;
            controller.sendStore(controller.free_head, node);

            state = NCAllocate_STORE;
            
            return false;
        case NCAllocate_STORE:
            controller.free_head = nextNodeId;
            current_pkt->makeResponse();
            current_pkt->deleteData();
            // TODO: consider returning status code
            return true;
        default:
            panic("incorrect state for node allocation operation!");
    }
    
}

bool
NodeControllerRevoke::transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) {
    return true;
}

bool
NodeControllerRcUpdate::transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) {
    return true;
}

void
NodeController::handleResp(PacketPtr pkt) {
    assert(currentPkt != NULL);

    NodeControllerCommand* cmd = currentPkt->getPtr<NodeControllerCommand>();
    assert(cmd != NULL);

    bool finish = cmd->transit(*this, currentPkt, pkt);

    delete pkt;

    if(finish) {
        pkt = currentPkt;
        currentPkt = NULL;

        cpu_side.trySendResp(pkt);
    }
}

Port& NodeController::getPort(const std::string& name, PortID idx) {
    if(name == "cpu_side")
        return cpu_side;
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
NodeController::nodeID2Addr(NodeID node_id) {
    return (Addr)(CAPSTONE_NODE_BASE_ADDR | (node_id * (sizeof(Node))));
}

void
NodeController::MemSidePort::trySendReq(PacketPtr pkt) {
    assert(retryPkt == NULL);
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
NodeControllerQuery::setup(NodeController& controller, PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "Read from node cache\n");

    controller.sendLoad(nodeId);
}


void
NodeControllerAllocate::setup(NodeController& controller, PacketPtr pkt) {
    // TODO: need to handle the case when there are no free nodes
    assert(controller.free_head != NODE_ID_INVALID);

    // fetch the free node
    state = NCAllocate_LOAD;
    controller.sendLoad(controller.free_head);
}

void
NodeControllerRcUpdate::setup(NodeController& controller, PacketPtr pkt) {
}

void
NodeControllerRevoke::setup(NodeController& controller, PacketPtr pkt) {
}


bool
NodeController::handleReq(PacketPtr pkt) {
    if(currentPkt != NULL) {
        return false;
    }
    
    NodeControllerCommand* cmd = pkt->getPtr<NodeControllerCommand>();
    assert(cmd != NULL);
    cmd->setup(*this, pkt);

    currentPkt = pkt;

    return true;
}


bool NodeController::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "NCache packet received\n");
    // no latency
    if(!owner->handleReq(pkt)) {
        toRetryReq = true;
        return false;
    }
    return true;
}


void NodeController::CPUSidePort::recvRespRetry() {
    assert(retryPkt != NULL);
    PacketPtr pkt = retryPkt;
    retryPkt = NULL;
    trySendResp(pkt);
}

void NodeController::CPUSidePort::trySendResp(PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "NCacheController try sending response\n");
    assert(retryPkt == NULL);
    if(!sendTimingResp(pkt)) {
        retryPkt = pkt;
    } else if(toRetryReq){
        toRetryReq = false;
        sendRetryReq(); // ready to receive new request
    }
}


void NodeController::CPUSidePort::recvFunctional(PacketPtr pkt) {
    recvTimingReq(pkt);
}


AddrRangeList NodeController::CPUSidePort::getAddrRanges() const {
    return std::list<AddrRange> { AddrRange(0, 0xffffffffLL) };
}

void
NodeController::functionalSetNodeValid(NodeID node_id, bool valid) {
    Addr naddr = nodeID2Addr(node_id);
    RequestPtr req = std::make_shared<Request>();
    req->requestorId(requestorId);
    req->setPaddr(naddr);
    PacketPtr pkt = Packet::createWrite(req);
    pkt->setSize(sizeof(Node));
    pkt->allocate();
    memset(pkt->getPtr<void>(), 0, sizeof(Node));
    *(pkt->getPtr<char>()) = (char)valid;
    mem_side.sendFunctional(pkt);
    assert(pkt->isResponse());
    delete pkt;
}

    
void
NodeController::allocObject(const AddrRange& obj) {
    functionalSetNodeValid((NodeID)objectRanges.size(), true);
    objectRanges.push_back(obj);
}

void
NodeController::freeObject(Addr addr) {
    std::optional<NodeID> node_id = lookupAddr(addr);
    assert(node_id);
    functionalSetNodeValid(node_id.value(), false);
}

void
NodeController::removeObject(Addr addr) {
    objectRanges.remove_if([addr](auto obj) { return obj.contains(addr); });
    // FIXME update the valid list
}

std::optional<NodeID>
NodeController::lookupAddr(Addr addr) {
    NodeID n = 0;
    for(auto& obj : objectRanges) {
        if(obj.contains(addr)){
            return std::optional<NodeID>(n);
        }
        ++ n;
    }
    return std::optional<NodeID>();
}

void
NodeController::regStats() {
    ClockedObject::regStats();
}


} // end of namespace gem5::RiscvcapstoneISA

