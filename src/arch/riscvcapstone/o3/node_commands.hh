#ifndef __CAPSTONE_NODE_COMMANDS_H_
#define __CAPSTONE_NODE_COMMANDS_H_

#include <memory>
#include "arch/riscvcapstone/types.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class NodeCommandCondition;
class LSQRequest;

/**
 * base class for all commands to node controller
 * */
struct NodeCommand {
    typedef enum {
        ALLOCATE,
        QUERY,
        RC_UPDATE,
        REVOKE,
        DROP
    } Type;
    DynInstPtr inst;
    bool canWB;
    std::unique_ptr<NodeCommandCondition> condition;
    // the returned data and size
    NodeCommand() : inst(NULL), canWB(false) {}
    NodeCommand(DynInstPtr inst) : inst(inst), canWB(false) {}
    virtual Type getType() const = 0;
    virtual bool readOnly() const = 0;
};

struct NodeQuery : NodeCommand {
    NodeID nodeId;
    Node* data = NULL;
    NodeQuery() {}
    NodeQuery(NodeID node_id) : nodeId(node_id) {}
    NodeQuery(DynInstPtr inst, NodeID node_id) : 
        NodeCommand(inst),
        nodeId(node_id) {}
    Type getType() const override;
    bool readOnly() const override;
    ~NodeQuery();
};


struct NodeRevoke : NodeCommand {
    NodeID nodeId;
    NodeRevoke() {}
    NodeRevoke(NodeID node_id) : nodeId(node_id) {}
    NodeRevoke(DynInstPtr inst, NodeID node_id) : 
        NodeCommand(inst),
        nodeId(node_id) {}
    Type getType() const override;
    bool readOnly() const override;
    ~NodeRevoke() {}
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

struct NodeRcUpdate : NodeCommand {
    NodeID nodeId;
    int delta;
    NodeRcUpdate() {}
    NodeRcUpdate(NodeID node_id, int delta): nodeId(node_id), delta(delta) {}
    NodeRcUpdate(DynInstPtr inst, NodeID node_id, int delta):
        NodeCommand(inst),
        nodeId(node_id), delta(delta) {}
    bool readOnly() const override;
    Type getType() const override;
    ~NodeRcUpdate() {}
    private:
        enum {
            NCRcUpdate_LOAD,
            NCRcUpdate_STORE,
        } state;
};

struct NodeAllocate : NodeCommand {
    NodeID data;

    NodeID parentId;
    NodeAllocate() {}
    NodeAllocate(NodeID parent_id): parentId(parent_id) {}
    NodeAllocate(DynInstPtr inst, NodeID parent_id):
        NodeCommand(inst),
        parentId(parent_id) {}
    bool readOnly() const override;
    Type getType() const override;
    ~NodeAllocate() {}
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

struct NodeDrop : NodeCommand {
    bool readOnly() const override;
    Type getType() const override;
};

typedef NodeCommand* NodeCommandPtr;

class NodeCommandCondition {
    public:
        virtual LSQRequest* getRequest() = 0;
        virtual bool satisfied() = 0;
};

}
}
}


#endif

