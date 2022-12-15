#ifndef __CAPSTONE_O3_NODE_CONTROLLER_H_
#define __CAPSTONE_O3_NODE_CONTROLLER_H_

#include "base/types.hh"
#include "arch/riscvcapstone/types.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

const int CAPSTONE_NODE_N = 1<<24;

// this does nothing more than maintaining some on-chip states
// related to the revocation tree
class NodeController {
    private:
        NodeID treeRoot; // root of the revocation tree
        NodeID freeHead; // head of the free list
        int freeNodeInited; 

    public:
        NodeController() : treeRoot(NODE_ID_INVALID),
            freeHead(NODE_ID_INVALID), 
            freeNodeInited(1) {} // node 0 is reserved for the lock
        void freeNode(Node& node, const NodeID& node_id);
        void setRoot(const NodeID& node_id) {
            treeRoot = node_id;
        }
        NodeID getRoot() const {
            return treeRoot;
        }
        NodeID tryAllocate(); // allocate from free list or uninited pool
};

}
}
}


#endif

