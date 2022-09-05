#ifndef NODE_CONTROLLER_H
#define NODE_CONTROLLER_H

#include<bitset>
#include<utility>
#include<optional>
#include<vector>
#include "sim/clocked_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/NodeController.hh"
#include "base/trace.hh"

#define NODE_CACHE_BLOCK_WIDTH 9
#define NODE_CACHE_LINES 4

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

        Stats::Scalar hits;
        Stats::Scalar misses;
        Stats::Formula hitRatio;

        
        CPUSidePort cpu_side;

        AddrRangeList objectRanges;
        std::vector<bool> objectValid;
        std::pair<NodeID, std::bitset<(1 << NODE_CACHE_BLOCK_WIDTH)> > nodeCacheLines[1 << NODE_CACHE_LINES];

        void setNodeValid(NodeID node_id, bool valid);
        bool queryNodeValid(NodeID node_id);
        void fetchCacheBlock(NodeID block); 

    public:
        NodeController(const NodeControllerParams& p);
        Port& getPort(const std::string& name, PortID idx) override;

        void allocObject(const AddrRange& obj);
        void freeObject(Addr addr);
        void removeObject(Addr addr);
        std::optional<NodeID> lookupAddr(Addr addr);

        void regStats() override;

};

} // end of namespace gem5::RiscvcapstoneISA

#endif

