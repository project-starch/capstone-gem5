#include "arch/riscvcapstone/ncache_cpu.hh"

namespace gem5::RiscvcapstoneISA {

Port& TimingSimpleNCacheCPU::getPort(const std::string& name, PortID idx) {
    if(name == "ncache_port")
        return ncache_port;
    return TimingSimpleCPU::getPort(name, idx);
}

void TimingSimpleNCacheCPU::NCacheRespTickEvent::schedule(
        PacketPtr pkt,
        Cycles cycles) {
    this->pkt = pkt;
    cpu->schedule(this, cpu->clockEdge(cycles));
}

void TimingSimpleNCacheCPU::NCacheRespTickEvent::process() {
    completeNCacheLoad(pkt);
}

bool TimingSimpleNCacheCPU::NCachePort::recvTimingResp(PacketPtr pkt) {
    if(tickEvent.scheduled()){
        if(!retryEvent.scheduled())
            cpu->schedule(&retryEvent, cpu->clockEdge(Cycles(1)));
        return false;
    }
    tickEvent.schedule(pkt); // handle the data at the next CPU cycle
    return true;
}

void TimingSimpleNCacheCPU::completeNCacheLoad(PacketPtr pkt) {
    assert(nodeResps.empty() || dataResps.empty());
    if(!dataResps.empty()){
        PacketPtr data_pkt = dataResps.front();
        dataResps.pop_front();
        completeDataAccess(data_pkt, pkt);
    } else{
        nodeResps.push_back(pkt);
    }
}

void TimingSimpleNCacheCPU::completeDataAccess(PacketPtr pkt) {
    assert(nodeResps.empty() || dataResps.empty());
    if(!nodeResps.empty()){
        PacketPtr node_pkt = nodeResps.front();
        nodeResps.pop_front();
        completeDataAccess(pkt, node_pkt);
    } else{
        dataResps.push_back(pkt);
    }
}

void TimingSimpleNCacheCPU::completeDataAccess(
        PacketPtr data_pkt,
        PacketPtr node_pkt) {
    TimingSimpleCPU::completeDataAccess(data_pkt);
}
    
}

