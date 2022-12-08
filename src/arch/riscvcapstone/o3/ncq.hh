#ifndef __CAPSTONE_NODE_COMMAND_QUEUE_H_
#define __CAPSTONE_NODE_COMMAND_QUEUE_H_

#include <vector>
#include <list>
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "arch/riscvcapstone/o3/limits.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/ncq_unit.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class CPU;

    // TODO: also needs to be one per thread
class NCQ {
    private:
        CPU* cpu;
        std::vector<NCQUnit> threads;
        int queueSize;
        int threadNum;
        std::list<ThreadID>* activeThreads;

    public:
        NCQ(CPU* cpu, int queue_size, int thread_num);
        void insertInstruction(const DynInstPtr& inst);
        void tick();

        bool isFull(ThreadID thread_id);

        Fault pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd);

        Fault executeNodeOp(const DynInstPtr& inst);

        void commitBefore(InstSeqNum seq_num, ThreadID thread_id);
        void writebackCommands();
        void writebackCommands(ThreadID thread_id);
        
        void setActiveThreads(std::list<ThreadID>* active_threads) {
            activeThreads = active_threads;
        }
    
};

}
}
}

#endif

