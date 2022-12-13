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

}
}
}
