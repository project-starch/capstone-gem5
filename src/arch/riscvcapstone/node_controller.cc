#include "arch/riscvcapstone/node_controller.hh"
#include "base/trace.hh"
#include "debug/CapstoneNCache.hh"

namespace gem5::RiscvcapstoneISA {

NodeController::NodeController(const NodeControllerParams& p) :
    ClockedObject(p),
    cpu_side(this) {
}

NodeController::CPUSidePort::CPUSidePort(NodeController* owner) :
    ResponsePort(owner->name() + ".cpu_side", owner),
    owner(owner),
    retryPkt(NULL) {

}

Port& NodeController::getPort(const std::string& name, PortID idx) {
    if(name == "cpu_side")
        return cpu_side;
    return ClockedObject::getPort(name, idx);
}


bool NodeController::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "NCache packet received\n");
    pkt->setSize(sizeof(uint64_t));
    pkt->allocate();
    
    std::optional<NodeID> node_id = owner->lookupAddr(pkt->getAddr());
    assert(!node_id);
    uint64_t resp_data = 1; 
    pkt->setData((uint8_t*)&resp_data);
    pkt->makeResponse();
    
    // no latency
    trySendResp(pkt);
    
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
    }
}


void NodeController::CPUSidePort::recvFunctional(PacketPtr pkt) {
    recvTimingReq(pkt);
}


AddrRangeList NodeController::CPUSidePort::getAddrRanges() const {
    return std::list<AddrRange> { AddrRange(0, 0xffffffffLL) };
}

std::optional<NodeController::NodeID>
NodeController::lookupAddr(Addr addr) {
    NodeID n = 0;
    for(auto& obj : object_ranges) {
        if(obj.contains(addr)){
            return std::optional<NodeController::NodeID>(n);
        }
        ++ n;
    }
    return std::optional<NodeController::NodeID>();
}

void
NodeController::addObject(const AddrRange& obj) {
    object_ranges.push_front(obj);
}

} // end of namespace gem5::RiscvcapstoneISA

