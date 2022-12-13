#ifndef __CAPSTONE_O3_NODE_CONTROLLER_H_
#define __CAPSTONE_O3_NODE_CONTROLLER_H_

#include "base/types.hh"
#include "arch/riscvcapstone/types.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


// this does nothing more than maintaining some on-chip states
// related to the revocation tree
class NodeController {
    private:
        NodeID treeRoot; // root of the revocation tree
        NodeID freeHead; // head of the free list
    public:
        void freeNode(Node& node, NodeID node_id);
};

}
}
}


#endif

