#ifndef __CAPSTONE_NCQ_UNIT_H_
#define __CAPSTONE_NCQ_UNIT_H_

#include <vector>
#include "cpu/inst_seq.hh"
#include "base/types.hh"
#include "base/circular_queue.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


struct NCQEntry {
    DynInstPtr inst;
    std::vector<NodeCommandPtr> commands;

    bool canWB;
    
    NCQEntry() {}

    NCQEntry(const DynInstPtr& inst) :
        inst(inst),
        canWB(false) {}
};

/**
 * Node command queue unit for a single thread
 *
 * */
class NCQUnit {
    private:

        CircularQueue<NCQEntry> ncQueue;
        int threadId;
        int queueSize;

    public:
        typedef typename CircularQueue<NCQEntry>::iterator NCQIterator;

        NCQUnit(ThreadID thread_id, int queue_size);
        //NCQUnit(const NCQUnit&) = delete;
        Fault pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd);
        void insertInstruction(const DynInstPtr& inst);
        // commit all instructions before specified seq number
        void commitBefore(InstSeqNum seq_num);
        void writebackCommands();
        void completeCommand(NCQIterator cmd_it);
        void tick();
        bool isFull();
    
};

}
}
}

#endif

