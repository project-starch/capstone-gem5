#include<cstring>
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
    system(p.system) {
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
NodeController::handleResp(PacketPtr pkt) {
    assert(currentPkt != NULL);

    currentPkt->makeResponse();
    assert(currentPkt->getPtr<NodeControllerCommand>() != NULL);
    currentPkt->deleteData();
    currentPkt->setSize(CAPSTONE_NODE_SIZE >> 3);
    currentPkt->allocate(); // TODO: let the cpu allocate
    memcpy(currentPkt->getPtr<void>(), pkt->getPtr<void>(), CAPSTONE_NODE_SIZE >> 3);

    delete pkt;

    pkt = currentPkt;
    currentPkt = NULL;
    
    cpu_side.trySendResp(pkt);
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
    return (Addr)(CAPSTONE_NODE_BASE_ADDR | (node_id * (CAPSTONE_NODE_SIZE >> 3)));
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
NodeController::setupQuery(const NodeControllerQuery& query) {
    DPRINTF(CapstoneNCache, "Read from node cache\n");

    Addr naddr = nodeID2Addr(query.nodeId);
    RequestPtr node_req = std::make_shared<Request>();
    node_req->requestorId(requestorId);
    node_req->setPaddr(naddr);
    PacketPtr node_pkt = Packet::createRead(node_req);
    node_pkt->setSize(CAPSTONE_NODE_SIZE >> 3); // FIXME: do we need to specify the size here?
    node_pkt->allocate();

    mem_side.trySendReq(node_pkt);
}


void
NodeController::setupAllocate(const NodeControllerAllocate& allocate) {
}

void
NodeController::setupRcUpdate(const NodeControllerRcUpdate& rc_update) {
}

void
NodeController::setupRevoke(const NodeControllerRevoke& revoke) {
}


bool
NodeController::handleReq(PacketPtr pkt) {
    if(currentPkt != NULL) {
        return false;
    }
    
    NodeControllerCommand* cmd = pkt->getPtr<NodeControllerCommand>();
    assert(cmd != NULL);
    switch(cmd->type) {
        case NodeControllerCommandType::NODE_ALLOCATE:
            setupAllocate(cmd->content.allocate);
            break;
        case NodeControllerCommandType::NODE_REVOKE:
            setupRevoke(cmd->content.revoke);
            break;
        case NodeControllerCommandType::NODE_RC_UPDATE:
            setupRcUpdate(cmd->content.rcUpdate);
            break;
        case NodeControllerCommandType::NODE_QUERY:
            setupQuery(cmd->content.query);
            break;
        default:
            ;
    }

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
    pkt->setSize(CAPSTONE_NODE_SIZE >> 3);
    pkt->allocate();
    memset(pkt->getPtr<void>(), 0, CAPSTONE_NODE_SIZE >> 3);
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

