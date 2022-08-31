#include "arch/riscvcapstone/node_controller.hh"

namespace gem5::RiscvcapstoneISA {

NodeController::CPUSidePort::CPUSidePort(NodeController* owner) :
    ClockedObject(owner->name() + ".cpu_side", owner),
    owner(owner),
    retryPkt(NULL) {

}

Port& NodeController::getPort(const std::string& name, PortID idx) {
    if(name == "cpu_side")
        return cpu_side;
    return ClockedObject::getPort(name, idx);
}


bool NodeController::CPUSidePort::recvTimingReq(PacketPtr pkt) {
    // TODO: just return dummy result for now

}


void NodeController::CPUSidePort::recvRespRetry() {
    assert(retryPkt != NULL);
    PacketPtr pkt = retryPkt;
    retryPkt = NULL;
    trySendResp(pkt);
}

void NodeController::CPUSidePort::trySendResp(PacketPtr pkt) {
    assert(retryPkt == NULL);
    if(!sendTimingResp(pkt)) {
        retryPkt = pkt;
    }
}

} // end of namespace gem5::RiscvcapstoneISA

