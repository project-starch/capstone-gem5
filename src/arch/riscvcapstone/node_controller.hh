#ifndef NODE_CONTROLLER_H
#define NODE_CONTROLLER_H

#include<optional>
#include<bitset>
#include "sim/clocked_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/NodeController.hh"
#include "base/trace.hh"

#define NODE_CONTROLLER_TAB_N 1024

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

        AddrRangeList objectRanges;
        std::bitset<NODE_CONTROLLER_TAB_N> objectValid;

    public:
        NodeController(const NodeControllerParams& p);
        Port& getPort(const std::string& name, PortID idx) override;

        void allocObject(const AddrRange& obj);
        void freeObject(Addr addr);
        void removeObject(Addr addr);
        std::optional<NodeID> lookupAddr(Addr addr);

};

} // end of namespace gem5::RiscvcapstoneISA

#endif

