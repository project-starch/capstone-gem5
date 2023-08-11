#ifndef __CAPSTONE_NODE_COMMANDS_H_
#define __CAPSTONE_NODE_COMMANDS_H_

#include <memory>
#include <utility>
#include "cpu/thread_context.hh"
#include "arch/riscvcapstone/types.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/node.hh"
#include "arch/riscvcapstone/o3/lsq.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class NodeCommandCondition;
class NCQ;
struct NCQEntry;

/**
 * base class for all commands to node controller
 * */
struct NodeCommand {
    typedef enum {
        REVOKE = 0,
        DROP = 1,
        ALLOCATE = 2,
        QUERY = 3,
        RC_UPDATE = 4
    } Type;

    typedef enum {
        NOT_STARTED,
        AWAIT_CACHE, // waiting for cache response
        TO_RESUME, // ready to resume
        COMPLETED
    } Status;

    Status status;

    DynInstPtr inst;
    bool canWB;
    std::unique_ptr<NodeCommandCondition> condition;
    // the returned data and size
    NodeCommand() : status(NOT_STARTED), inst(NULL), canWB(false) {}
    NodeCommand(DynInstPtr inst) : status(NOT_STARTED), inst(inst), canWB(false) {}
    virtual ~NodeCommand() {}
    virtual Type getType() const = 0;
    virtual bool beforeCommit() const = 0; // if this command can be executed before commit
    virtual PacketPtr transition() = 0;
    virtual void handleResp(PacketPtr pkt) = 0;
    virtual bool error() {
        return false;
    }
    virtual void setInst(const DynInstPtr& inst) {
        this->inst = inst;
    }
    virtual void dump() {

    }

    protected:
        PacketPtr createStoreNode(const NodeID& node_id, const Node& node);
        PacketPtr createLoadNode(const NodeID& node_id);
};


/**
 * Node command that requires locking
 * */
struct LockedNodeCommand : NodeCommand {
    std::unique_ptr<NodeCommand> rawCommand;

    typedef enum {
        BEFORE_ACQUIRE,
        ACQUIRED,
        RELEASED 
    } LockState;

    LockState lockState;

    LockedNodeCommand(std::unique_ptr<NodeCommand> raw_command) :
        rawCommand(std::move(raw_command))
    {
    }

    Type getType() const override {
        return rawCommand->getType();
    }

    bool beforeCommit() const override {
        return rawCommand->beforeCommit();
    }

    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;

    void setInst(const DynInstPtr& inst) override {
        NodeCommand::setInst(inst);
        rawCommand->setInst(inst);
    }

    private:
        PacketPtr createAcquirePacket();
        PacketPtr createReleasePacket();
};

struct NodeQuery : NodeCommand {
    NodeID nodeId;
    bool validityError = false;
    
    NodeQuery() {}
    NodeQuery(NodeID node_id) : nodeId(node_id) {}
    NodeQuery(DynInstPtr inst, NodeID node_id) : 
        NodeCommand(inst),
        nodeId(node_id) {}
    Type getType() const override {
        return NodeCommand::QUERY;
    }
    bool beforeCommit() const override {
        return true;
    }
    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;
    bool error() override;
    ~NodeQuery() {}
};

struct NodeQueryDbg : NodeCommand {
    NodeID nodeId;
    bool validityError = false;
    
    NodeQueryDbg() {}
    NodeQueryDbg(NodeID node_id) : nodeId(node_id) {}
    NodeQueryDbg(DynInstPtr inst, NodeID node_id) : 
        NodeCommand(inst),
        nodeId(node_id) {}
    Type getType() const override {
        return NodeCommand::QUERY;
    }
    bool beforeCommit() const override {
        return true;
    }
    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;
    bool error() override;
    ~NodeQueryDbg() {}
};


struct NodeRevoke : NodeCommand {
    NodeID nodeId;
    NodeRevoke() : state(NCRevoke_LOAD_ROOT) {}
    NodeRevoke(NodeID node_id) : nodeId(node_id), state(NCRevoke_LOAD_ROOT) {}
    NodeRevoke(DynInstPtr inst, NodeID node_id) : 
        NodeCommand(inst),
        nodeId(node_id), 
        state(NCRevoke_LOAD_ROOT) {}
    Type getType() const override {
        return NodeCommand::REVOKE;
    }
    bool beforeCommit() const override {
        return false;
    }
    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;
    ~NodeRevoke() {}
    private:
        enum {
            NCRevoke_LOAD_ROOT,
            NCRevoke_LOAD,
            NCRevoke_STORE,
            NCRevoke_STORE_RIGHT,
            NCRevoke_STORE_ROOT,
        } state;
        NodeID curNodeId, nextNodeId;
        unsigned int rootDepth;
        
        Node savedNode, rootNode;
};

struct NodeRcUpdate : NodeCommand {
    NodeID nodeId;
    int delta;
    NodeRcUpdate() : state(NCRcUpdate_LOAD) {}
    NodeRcUpdate(NodeID node_id, int delta): nodeId(node_id), delta(delta), state(NCRcUpdate_LOAD) {}
    NodeRcUpdate(DynInstPtr inst, NodeID node_id, int delta):
        NodeCommand(inst),
        nodeId(node_id), delta(delta), state(NCRcUpdate_LOAD) {}
    Type getType() const override {
        return NodeCommand::RC_UPDATE;
    }
    bool beforeCommit() const override {
        return false;
    }
    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;
    void dump() override;
    ~NodeRcUpdate() {}
    private:
        enum {
            NCRcUpdate_LOAD,
            NCRcUpdate_STORE,
        } state;
        
        Node savedNode;
};

struct NodeAllocate : NodeCommand {
    NodeID data;

    NodeID parentId;
    bool asChild;

    NodeAllocate() : state(NCAllocate_LOAD_PARENT) {}
    // If as_child is false, create a sibling to parent_id instead.
    // This requires parent_id to be a leaf node
    NodeAllocate(NodeID parent_id, NodeID to_allocate_id, bool as_child=true): parentId(parent_id), toAllocate(to_allocate_id), asChild(as_child),
    state(NCAllocate_LOAD_PARENT) {}
    NodeAllocate(DynInstPtr inst, NodeID parent_id, NodeID to_allocate_id):
        NodeCommand(inst),
        parentId(parent_id), toAllocate(to_allocate_id), 
        state(NCAllocate_LOAD_PARENT) {}
    Type getType() const override {
        return NodeCommand::ALLOCATE;
    }
    bool beforeCommit() const override {
        return false;
    }
    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;
    ~NodeAllocate() {}
    NodeID getAllocatedID() {
        return toAllocate;
    }
    private:
        enum { 
            NCAllocate_LOAD_PARENT,
            NCAllocate_STORE_PARENT,
            NCAllocate_LOAD,
            NCAllocate_STORE,
            NCAllocate_LOAD_RIGHT,
            NCAllocate_STORE_RIGHT,
        } state;
        NodeID nextNodeId, nextFreeNodeId, toAllocate;
        unsigned int parentDepth;
        Node savedNode;
};

struct NodeDrop : NodeCommand {
    NodeID nodeId;

    NodeDrop() {}
    NodeDrop(NodeID node_id): nodeId(node_id) {}

    PacketPtr transition() override;
    void handleResp(PacketPtr pkt) override;
    Type getType() const override {
        return NodeCommand::DROP;
    }
    bool beforeCommit() const override {
        return false;
    }
    private:
        enum {
            NCDrop_LOAD,
            NCDrop_STORE,
            NCDrop_LOAD_LEFT,
            NCDrop_STORE_LEFT,
            NCDrop_LOAD_RIGHT,
            NCDrop_STORE_RIGHT,
        } state = NCDrop_LOAD;

        Node savedNode;
        NodeID prevNodeId;
        NodeID nextNodeId;
};

typedef NodeCommand* NodeCommandPtr;

class NodeCommandCondition {
    public:
        virtual LSQ::LSQRequest* getRequest() = 0; // TODO: perhaps to remove
        virtual bool satisfied(LSQ::LSQRequest* req) = 0;
};

}
}
}


#endif

