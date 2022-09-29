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
#include "arch/riscvcapstone/cap_track.hh"

// size of each revocation nodes (in bits)

namespace gem5::RiscvcapstoneISA {

const size_t CAPSTONE_NODE_SIZE = 128;

class NodeController;

const NodeID NODE_ID_INVALID = (NodeID)(-1ULL & ((1ULL << 31) - 1));

/**
 * base class for all commands to node controller
 * */
struct NodeControllerCommand {
    virtual void setup(NodeController& controller, PacketPtr pkt) = 0;
    virtual bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) = 0;
    virtual Tick handleAtomic(NodeController& controller, PacketPtr pkt) = 0;
};

struct NodeControllerQuery : NodeControllerCommand {
    NodeID nodeId;
    NodeControllerQuery() {}
    NodeControllerQuery(NodeID node_id) : nodeId(node_id) {}
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
    Tick handleAtomic(NodeController& controller, PacketPtr pkt) override;
};


struct NodeControllerRevoke : NodeControllerCommand {
    NodeID nodeId;
    NodeControllerRevoke() {}
    NodeControllerRevoke(NodeID node_id) : nodeId(node_id) {}
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
    Tick handleAtomic(NodeController& controller, PacketPtr pkt) override;
    private:
        enum {
            NCRevoke_LOAD_ROOT,
            NCRevoke_LOAD,
            NCRevoke_STORE,
            NCRevoke_STORE_RIGHT,
            NCRevoke_LOAD_LEFT, NCRevoke_STORE_LEFT,
        } state;
        NodeID curNodeId;
        unsigned int rootDepth;
        NodeID prevNodeId;
};

struct NodeControllerRcUpdate : NodeControllerCommand {
    NodeID nodeId;
    int delta;
    NodeControllerRcUpdate() {}
    NodeControllerRcUpdate(NodeID node_id, int delta): nodeId(node_id), delta(delta) {}
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
    Tick handleAtomic(NodeController& controller, PacketPtr pkt) override;
    private:
        enum {
            NCRcUpdate_LOAD,
            NCRcUpdate_STORE,
        } state;
};

struct NodeControllerAllocate : NodeControllerCommand {
    NodeID parentId;
    NodeControllerAllocate() {}
    NodeControllerAllocate(NodeID parent_id): parentId(parent_id) {}
    void setup(NodeController& controller, PacketPtr pkt) override;
    bool transit(NodeController& controller, PacketPtr current_pkt, PacketPtr pkt) override;
    Tick handleAtomic(NodeController& controller, PacketPtr pkt) override;
    private:
        enum { 
            NCAllocate_LOAD,
            NCAllocate_STORE,
            NCAllocate_LOAD_PARENT,
            NCAllocate_STORE_PARENT,
            NCAllocate_LOAD_RIGHT,
            NCAllocate_STORE_RIGHT,
        } state;
        NodeID nextNodeId, nextFreeNodeId, toAllocate;
        bool fromFreeList;
        unsigned int parentDepth;
};

typedef NodeControllerCommand* NodeControllerCommandPtr;


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

struct SimpleAddrRange {
    Addr start, end;
    SimpleAddrRange(Addr start, Addr end) : start(start), end(end) {}
    bool operator < (const SimpleAddrRange& other) const {
        if(start != other.start) {
            return start < other.start;
        }
        return end < other.end;
    }

    bool operator == (const SimpleAddrRange& other) const {
        return start == other.start &&
            end == other.end;
    }

    bool contains(const Addr& addr) const {
        return start <= addr && addr < end;
    }
};

typedef std::set<SimpleAddrRange> SimpleAddrRangeSet;


class NodeController : public ClockedObject {
    private:
        class CPUSidePort : public ResponsePort {
            private:
                NodeController* owner;
                PacketPtr retryPkt;
                bool toRetryReq;

            public:
                CPUSidePort(NodeController* owner);
                Tick recvAtomic(PacketPtr pkt) override;
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

        SimpleAddrRangeSet objectRanges;
    
        System* system; // the system the node controller belongs to
        RequestorID requestorId;

        CapTrackMap capTrackMap;


        void sendPacketToMem(PacketPtr pkt, bool atomic);

    public:
        NodeController(const NodeControllerParams& p);
        Port& getPort(const std::string& name, PortID idx) override;

        // only update the object registry
        // does not touch anything else
        void allocObject(const SimpleAddrRange& obj);
        void freeObject(Addr addr);

        std::optional<SimpleAddrRange> lookupAddr(Addr addr);
        Addr nodeId2Addr(NodeID node_id);

        void regStats() override;

        bool handleTimingReq(PacketPtr pkt);
        Tick handleAtomicReq(PacketPtr pkt);
        void handleResp(PacketPtr pkt);

        void init() override;

        PacketPtr sendLoad(NodeID node_id, bool atomic = false);
        PacketPtr sendStore(NodeID node_id, const Node& node, bool atomic = false);

        void atomicLoadNode(NodeID node_id, Node* node) {
            PacketPtr pkt = sendLoad(node_id, true);
            memcpy(node, pkt->getPtr<void>(), sizeof(Node));
            delete pkt;
        }

        void atomicStoreNode(NodeID node_id, const Node* node) {
            PacketPtr pkt = sendStore(node_id, *node, true);
            delete pkt;
        }

        void addCapTrack(const CapLoc& loc, NodeID node_id);
        NodeID queryCapTrack(const CapLoc& loc);
        void removeCapTrack(const CapLoc& loc);

        void freeNode(Node& node, NodeID node_id);

        // free list
        NodeID free_head;
        // tree
        NodeID tree_root;

        int freeNodeInited;
};

} // end of namespace gem5::RiscvcapstoneISA

#endif

