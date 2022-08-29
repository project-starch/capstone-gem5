#include "learning_gem5/cache/simple_cache.hh"

namespace gem5 {

SimpleCache::SimpleCache(const SimpleCacheParams& params):
    ClockedObject(params),
    mem_side_port(params.name + ".mem_side", this),
    blocked(false) {
    for(int i = 0; i < params.port_cpu_side_connection_count; i ++){
        cpu_side_ports.emplace_back(params.name + csprintf(".cpu_side[%d]", i),
                (PortID)i, this);
    }
}

Port& SimpleCache::getPort(const std::string& if_name, PortID idx) {
    if(if_name == "cpu_side") {
        return cpu_side_ports[idx];
    } else if(if_name == "mem_side"){
        return mem_side_port;
    } else{
        panic("trying to access a port that does not exist.");
    }
}

void SimpleCache::handleFunctional(PacketPtr pkt) {
    // dummy for now
    mem_side_port.sendFunctional(pkt);
}

AddrRangeList SimpleCache::getAddrRanges() const {
    return mem_side_port.getAddrRanges();
}

void SimpleCache::handleRangeChange() {
    for(auto& p: cpu_side_ports) {
        p.sendRangeChange();
    }
}

bool SimpleCache::handleTimingReq(PacketPtr pkt, PortID from_port) {
    if(blocked)
        return false;
    blocked = true;
    blocked_port = from_port; 

    mem_side_port.sendPacket(pkt); // this won't block (though it might require retry)
    return true;
}

void SimpleCache::CPUSidePort::recvFunctional(PacketPtr pkt) {
    owner->handleFunctional(pkt);
}

bool SimpleCache::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    assert(!pending_send_retry); // before a retry is sent please don't try again
    if(!owner->handleTimingReq(pkt, idx)) { // still handling a packet
        pending_send_retry = true;
        return false;
    }
    return true;
}

void SimpleCache::CPUSidePort::sendPacket(PacketPtr pkt) {
    assert(retry_send_pkt == nullptr);
    if(!sendTimingResp(pkt))
        retry_send_pkt = pkt;
}

void SimpleCache::MemSidePort::sendPacket(PacketPtr pkt) {
    assert(retry_send_pkt == nullptr);
    if(!sendTimingReq(pkt))
        retry_send_pkt = pkt;
}

void SimpleCache::MemSidePort::recvRangeChange() {
    owner->handleRangeChange();
}

void SimpleCache::CPUSidePort::recvRespRetry() {
    assert(retry_send_pkt != nullptr);
    PacketPtr pkt = retry_send_pkt;
    retry_send_pkt = nullptr;
    sendPacket(pkt);
}

// simply try resending the packet
void SimpleCache::MemSidePort::recvReqRetry() {
    assert(retry_send_pkt != nullptr);
    PacketPtr pkt = retry_send_pkt;
    retry_send_pkt = nullptr;
    sendPacket(pkt);
}

void SimpleCache::CPUSidePort::trySendRetry() {
    if(pending_send_retry) {
        pending_send_retry = false;
        sendRetryReq();
    }
}

bool SimpleCache::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleTimingResp(pkt);
}

// protocol: when a slave port needs to send a response, it will not receive a request
// hence here we can safely hand response over to the slave port to be sent
bool SimpleCache::handleTimingResp(PacketPtr pkt) {
    assert(blocked);
    blocked = false;

    cpu_side_ports[blocked_port].sendPacket(pkt);
    
    for(auto& p: cpu_side_ports) {
        p.trySendRetry();
    }
    
    // the slave port is always able to accept the packet
    return true;
}

AddrRangeList SimpleCache::CPUSidePort::getAddrRanges() const {
    return owner->getAddrRanges();
}

}

