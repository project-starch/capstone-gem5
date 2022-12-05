#ifndef __CAPSTONE_NODE_COMMANDS_H_
#define __CAPSTONE_NODE_COMMANDS_H_

#include "arch/riscvcapstone/types.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

/**
 * base class for all commands to node controller
 * */
struct NodeCommand {
    typedef enum {
        ALLOCATE,
        QUERY,
        RC_UPDATE,
        REVOKE
    } Type;
    DynInst* insn;
    bool canWB;
    // the returned data and size
    NodeCommand() : insn(NULL), canWB(false) {}
    NodeCommand(DynInst* insn) : insn(insn), canWB(false) {}
    virtual Type getType() const = 0;
    virtual bool readOnly() const = 0;
};

struct NodeQuery : NodeCommand {
    NodeID nodeId;
    Node* data = NULL;
    NodeQuery() {}
    NodeQuery(NodeID node_id) : nodeId(node_id) {}
    NodeQuery(DynInst* insn, NodeID node_id) : 
        NodeCommand(insn),
        nodeId(node_id) {}
    Type getType() const override;
    bool readOnly() const override;
    ~NodeQuery();
};


struct NodeRevoke : NodeCommand {
    NodeID nodeId;
    NodeRevoke() {}
    NodeRevoke(NodeID node_id) : nodeId(node_id) {}
    NodeRevoke(DynInst* insn, NodeID node_id) : 
        NodeCommand(insn),
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
    NodeRcUpdate(DynInst* insn, NodeID node_id, int delta):
        NodeCommand(insn),
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
    NodeAllocate(DynInst* insn, NodeID parent_id):
        NodeCommand(insn),
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

typedef NodeCommand* NodeCommandPtr;

}
}
}


#endif

