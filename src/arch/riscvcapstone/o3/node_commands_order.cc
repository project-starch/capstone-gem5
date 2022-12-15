#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/node_commands_order.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

#if(0)
const bool NODE_COMMAND_ORDERING_MAP[5][5] = {
    {false, false, false, false, true},
    {false, false, false, false, true},
    {false, false, false, true, true},
    {false, false, true, true, true},
    {true, true, true, true, true}
};
#else
const bool NODE_COMMAND_ORDERING_MAP[5][5] = { // always in order
    {false, false, false, false, false},
    {false, false, false, false, false},
    {false, false, false, false, false},
    {false, false, false, false, false},
    {false, false, false, false, false}
};
#endif

bool
NodeCommandsOrdering::reorderAllowed(
        NodeCommandPtr cmd_a,
        NodeCommandPtr cmd_b) const {
    return NODE_COMMAND_ORDERING_MAP[
        static_cast<int>(cmd_a->getType())
    ][
        static_cast<int>(cmd_b->getType())
    ];
}

}
}
}
