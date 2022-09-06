#include "arch/riscvcapstone/node_controller.hh"
#include "base/trace.hh"
#include "debug/CapstoneNCache.hh"

namespace gem5::RiscvcapstoneISA {

NodeController::NodeController(const NodeControllerParams& p) :
    ClockedObject(p),
    cpu_side(this) {
    // mark all cache lines as invalid
    for(auto& line : nodeCacheLines) {
        line.first = (NodeController::NodeID)(-1ULL);
    }
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
    uint64_t resp_data;
    if(node_id) {
        resp_data = (uint64_t)owner->queryNodeValid(node_id.value());
    } else{
        resp_data = 0;
    }
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

void
NodeController::setNodeValid(NodeController::NodeID node_id, bool valid) {
    assert(node_id <= (NodeController::NodeID)objectValid.size());
    if(node_id == (NodeController::NodeID)objectValid.size()) {
        objectValid.push_back(valid);
        for(NodeController::NodeID i = 1; i < (1 << NODE_CACHE_BLOCK_WIDTH); i ++)
            objectValid.push_back(false);
    } else{
        objectValid[node_id] = valid;
    }
    
    NodeController::NodeID block = node_id >> NODE_CACHE_BLOCK_WIDTH;
    fetchCacheBlock(block);
    nodeCacheLines[block & ((1 << NODE_CACHE_LINES) - 1)].second
        [node_id & ((1 << NODE_CACHE_BLOCK_WIDTH) - 1)] = valid;
}

bool
NodeController::queryNodeValid(NodeController::NodeID node_id) {
    assert(node_id < (NodeController::NodeID)objectValid.size());
    NodeController::NodeID block = node_id >> NODE_CACHE_BLOCK_WIDTH;
    fetchCacheBlock(block);
    return nodeCacheLines[block & ((1 << NODE_CACHE_LINES) - 1)].second
        [node_id & ((1 << NODE_CACHE_BLOCK_WIDTH) - 1)];
}

void
NodeController::fetchCacheBlock(NodeController::NodeID block) {
    NodeController::NodeID line = block & ((1 << NODE_CACHE_LINES) - 1);
    if(nodeCacheLines[line].first == (block >> NODE_CACHE_LINES)) {
        ++ hits;
    } else{
        nodeCacheLines[line].first = block >> NODE_CACHE_LINES;
        for(NodeController::NodeID i = 0; i < (1 << NODE_CACHE_BLOCK_WIDTH); i ++){
            nodeCacheLines[line].second[i] = objectValid[(block << NODE_CACHE_BLOCK_WIDTH) | i];
        }
        ++ misses;
    }
}

void
NodeController::allocObject(const AddrRange& obj) {
    setNodeValid((NodeController::NodeID)objectRanges.size(), true);
    objectRanges.push_back(obj);
}

void
NodeController::freeObject(Addr addr) {
    std::optional<NodeController::NodeID> node_id = lookupAddr(addr);
    if(node_id) {
        setNodeValid(node_id.value(), false);
    }
}

void
NodeController::removeObject(Addr addr) {
    objectRanges.remove_if([addr](auto obj) { return obj.contains(addr); });
    // FIXME update the valid list
}

std::optional<NodeController::NodeID>
NodeController::lookupAddr(Addr addr) {
    NodeID n = 0;
    for(auto& obj : objectRanges) {
        if(obj.contains(addr)){
            return std::optional<NodeController::NodeID>(n);
        }
        ++ n;
    }
    return std::optional<NodeController::NodeID>();
}

void
NodeController::regStats() {
    ClockedObject::regStats();

    hits.name(name() + ".hits").desc("Number of hits");
    misses.name(name() + ".misses").desc("Number of misses");
    hitRatio.name(name() + ".hitRatio").desc("Hit ratio");
    hitRatio = hits / (hits + misses);
}


} // end of namespace gem5::RiscvcapstoneISA

