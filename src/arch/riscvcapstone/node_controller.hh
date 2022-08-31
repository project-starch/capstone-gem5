#ifndef NODE_CONTROLLER_H
#define NODE_CONTROLLER_H

#include "clocked_object.hh"
#include "params/NodeController.hh"

namespace gem5::RiscvcapstoneISA {

class NodeController : public ClockedObject {
    private:
        class CPUSidePort : public ResponsePort {
            private:
                NodeController* owner;
                PacketPtr retryPkt;
            public:
                CPUSidePort(NodeController* owner);
                Tick recvAtomic(PacketPtr pkt) override {
                    panic("atomic mode unsupported.");
                }
                bool recvTimingReq(PacketPtr pkt) override;
                void recvRespRetry() override;
                void trySendResp(PacketPtr pkt);
        };
        
        CPUSidePort cpu_side;
    public:
        NodeController(const NodeControllerParams& p);
        Port& getPort(const std::string& name, PortID idx) override;
};

} // end of namespace gem5::RiscvcapstoneISA

#endif

