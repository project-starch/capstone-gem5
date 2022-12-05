#ifndef __CAPSTONE_NODE_COMMAND_QUEUE_H_
#define __CAPSTONE_NODE_COMMAND_QUEUE_H_


#include "base/circular_queue.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

    // TODO: also needs to be one per thread
class NCQ {
    private:
        CircularQueue<NodeCommandPtr> cmdQueue;
    public:
        NCQ(int size): cmdQueue(size) {}
        void insertInstruction(DynInstPtr inst);
        void tick();
};

}
}
}

#endif

