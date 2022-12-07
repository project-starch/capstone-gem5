#ifndef __CAPSTONE_NCQ_UNIT_H_
#define __CAPSTONE_NCQ_UNIT_H_

#include "base/types.hh"
#include "base/circular_queue.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


struct NCQEntry {
    DynInstPtr inst;
    NodeCommandPtr cmd;

    bool canWB;
    
    NCQEntry() {}

    NCQEntry(const DynInstPtr& inst) :
        inst(inst),
        cmd(nullptr),
        canWB(false) {}
};

/**
 * Node command queue unit for a single thread
 *
 * */
class NCQUnit {
    private:
        CircularQueue<NCQEntry> ncQueue;
        int queueSize;

    public:
        NCQUnit(int queue_size);
        Fault pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd);
        void insertInstruction(const DynInstPtr& inst);
        void tick();

        bool isFull() {
            return ncQueue.full();
        }
};

}
}
}

#endif

