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

namespace gem5::RiscvcapstoneISA {

const size_t CAPSTONE_NODE_SIZE = 128;

typedef uint64_t NodeID;

class NodeController;

const NodeID NODE_ID_INVALID = (NodeID)-1ULL;

struct NodeControllerCommand {
    virtual void setup(NodeController& controller, PacketPtr pkt) = 0;
    virtual bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) = 0;
};

struct NodeControllerQuery : NodeControllerCommand {
    NodeID nodeId;
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
};


struct NodeControllerRevoke : NodeControllerCommand {
    NodeID nodeId;
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
};

struct NodeControllerRcUpdate : NodeControllerCommand {
    NodeID nodeId;
    int delta;
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
};

struct NodeControllerAllocate : NodeControllerCommand {
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
    private:
        enum { 
            NCAllocate_LOAD,
            NCAllocate_STORE
        } state;
        NodeID nextNodeId;
};


/*
    revocation node structure
    State (2 bits)
    Counter (33 bit)
    Previous node (31 bits)
    Next node (31 bits)
    Depth (31 bits)
*/
struct Node {
    NodeID prev: 31;
    NodeID next: 31;
    unsigned char state: 2;
    unsigned long long counter: 33;
    unsigned int depth: 31;
};

static_assert(sizeof(Node) == (CAPSTONE_NODE_SIZE >> 3));


class NodeController : public ClockedObject {
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

        void sendLoad(NodeID node_id);
        void sendStore(NodeID node_id, const Node& node);

        // free list
        NodeID free_head;
        // tree
        NodeID tree_root;

};

} // end of namespace gem5::RiscvcapstoneISA

#endif

