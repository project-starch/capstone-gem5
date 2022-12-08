#ifndef __CAPSTONE_NODE_COMMANDS_ORDER_H_
#define __CAPSTONE_NODE_COMMANDS_ORDER_H_

#include "arch/riscvcapstone/o3/node_commands.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class NodeCommandsOrdering {
    public:
        bool reorderAllowed(NodeCommandPtr cmd_a,
                NodeCommandPtr cmd_b) const;
};

}
}
}

#endif


