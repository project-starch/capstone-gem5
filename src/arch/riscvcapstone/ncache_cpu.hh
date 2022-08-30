#ifndef NCACHE_CPU
#define NCACHE_CPU

#include<queue>
#include "cpu/simple/timing.hh"
#include "params/BaseTimingSimpleNCacheCPU.hh"

namespace gem5::RiscvcapstoneISA {

class TimingSimpleNCacheCPU: public TimingSimpleCPU {
    private:
        std::queue<PacketPtr> dataResps;
        std::queue<PacketPtr> nodeResps;
    
        void completeDataAccess(PacketPtr data_pkt, PacketPtr node_pkt);
    protected:
        class NCachePort: public RequestPort {
            private:
                struct NCacheRespTickEvent : public Event {
                    NCacheRespTickEvent(TimingSimpleNCacheCPU* cpu) :
                        cpu(cpu) {}
                    void schedule(PacketPtr pkt, Cycles cycles = Cycles(0));
                    void process() override;
                };

                TimingSimpleNCacheCPU* cpu;
                NCacheRespTickEvent tickEvent;
                EventFunctionWrapper retryEvent;

                PacketPtr pkt;
            public:
                NCachePort(TimingSimpleNCacheCPU* cpu):
                    RequestPort(cpu->name() + ".ncache_port", cpu),
                    cpu(cpu),
                    tickEvent(cpu),
                    retryEvent([this] { sendRetryResp(); }) {}
            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                bool recvReqRetry() override;
        };

        NCachePort ncache_port;

    public:
        TimingSimpleNCacheCPU(const BaseTimingSimpleNCacheCPUParams& p):
            TimingSimpleCPU(p),
            ncache_port(this) { }
        Port& getPort(const std::string& name, PortID idx) override;
        void completeDataAccess(PacketPtr pkt) override;

};

}

#endif

