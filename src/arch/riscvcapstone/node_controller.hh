#ifndef NODE_CONTROLLER_H
#define NODE_CONTROLLER_H

#include<optional>
#include "sim/clocked_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/NodeController.hh"
#include "base/trace.hh"

namespace gem5::RiscvcapstoneISA {

class NodeController : public ClockedObject {
    typedef uint64_t NodeID;
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
                void recvFunctional(PacketPtr pkt) override;
                AddrRangeList getAddrRanges() const override;
                void trySendResp(PacketPtr pkt);
        };
        
        CPUSidePort cpu_side;
        AddrRangeList object_ranges; // address ranges of allocated objects on heap

        std::optional<NodeID> lookupAddr(Addr addr);
    public:
        NodeController(const NodeControllerParams& p);
        Port& getPort(const std::string& name, PortID idx) override;
        
        void addObject(const AddrRange& obj);
};

} // end of namespace gem5::RiscvcapstoneISA

#endif

