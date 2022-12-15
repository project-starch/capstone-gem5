#include "arch/riscvcapstone/o3/node_controller.hh"
#include "debug/CapstoneNodeOps.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

void
NodeController::freeNode(Node& node, const NodeID& node_id) {
    //DPRINTF(CapstoneNodeOps, "free node with id %lu\n", node_id);
    node.next = freeHead;
    freeHead = node_id;
}

NodeID
NodeController::tryAllocate() {
    if(freeHead != NODE_ID_INVALID) {
        //from_free_list = true;
        return freeHead;
    }
    if(freeNodeInited >= CAPSTONE_NODE_N){
        return NODE_ID_INVALID;
    }
    //from_free_list = false;
    return static_cast<NodeID>(freeNodeInited ++);
}

}
}
}
