#ifndef NODE_CONTROLLER_H
#define NODE_CONTROLLER_H

#include<bitset>
#include<utility>
#include<optional>
#include<vector>
#include "sim/clocked_object.hh"
#include "sim/system.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/NodeController.hh"
#include "base/trace.hh"

// size of each revocation nodes (in bits)
#define CAPSTONE_NODE_SIZE 128

namespace gem5::RiscvcapstoneISA {

class NodeController : public ClockedObject {
    typedef uint64_t NodeID;
    private:
        class CPUSidePort : public ResponsePort {
            private:
                NodeController* owner;
                PacketPtr retryPkt;
                bool toRetryReq;

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

        class MemSidePort : public RequestPort {
            private:
                NodeController* owner;
                PacketPtr retryPkt; // request packet to retry

            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;

            public:
                MemSidePort(NodeController* owner);

                void trySendReq(PacketPtr pkt);
            
        };

        PacketPtr currentPkt; // packet currently handling
        
        CPUSidePort cpu_side;
        MemSidePort mem_side;

        AddrRangeList objectRanges;
    
        System* system; // the system the node controller belongs to
        RequestorID requestorId;

        void functionalSetNodeValid(NodeID node_id, bool valid);

    public:
        NodeController(const NodeControllerParams& p);
        Port& getPort(const std::string& name, PortID idx) override;

        void allocObject(const AddrRange& obj);
        void freeObject(Addr addr);
        void removeObject(Addr addr);
        std::optional<NodeID> lookupAddr(Addr addr);
        Addr nodeID2Addr(NodeID node_id);

        void regStats() override;

        bool handleReq(PacketPtr pkt);
        void handleResp(PacketPtr pkt);

        void init() override;

};

} // end of namespace gem5::RiscvcapstoneISA

#endif

