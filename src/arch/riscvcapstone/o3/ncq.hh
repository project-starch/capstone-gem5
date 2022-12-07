#ifndef __CAPSTONE_NODE_COMMAND_QUEUE_H_
#define __CAPSTONE_NODE_COMMAND_QUEUE_H_

#include <vector>
#include "base/types.hh"
#include "arch/riscvcapstone/o3/limits.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/ncq_unit.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

    // TODO: also needs to be one per thread
class NCQ {
    private:
        std::vector<NCQUnit> threads;
        int queueSize;
        int threadNum;

    public:
        NCQ(int queue_size, int thread_num);
        void insertInstruction(const DynInstPtr& inst);
        void tick();

        bool isFull(ThreadID thread_id);

        Fault pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd);
};

}
}
}

#endif

