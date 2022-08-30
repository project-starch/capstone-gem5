#ifndef NCACHE_CPU
#define NCACHE_CPU

#include<queue>
#include "cpu/simple/timing.hh"
#include "params/BaseTimingSimpleNCacheCPU.hh"

namespace gem5::RiscvcapstoneISA {

class TimingSimpleNCacheCPU: public TimingSimpleCPU {
    private:
    
        void completeDataAccess(PacketPtr data_pkt, PacketPtr node_pkt);
    protected:

    public:
        TimingSimpleNCacheCPU(const BaseTimingSimpleNCacheCPUParams& p):
            TimingSimpleCPU(p),
            ncache_port(this) { }
        Port& getPort(const std::string& name, PortID idx) override;
        void completeDataAccess(PacketPtr pkt) override;

};

}

#endif

