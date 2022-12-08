#ifndef __CAPSTONE_NCQ_UNIT_H_
#define __CAPSTONE_NCQ_UNIT_H_

#include <vector>
#include <unordered_map>
#include "cpu/inst_seq.hh"
#include "base/types.hh"
#include "base/circular_queue.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/node_commands_order.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class NCQ;

struct NCQEntry {
    DynInstPtr inst;
    // TODO: might consider maintaining different types of commands in
    // different queues later
    std::vector<NodeCommandPtr> commands;

    bool canWB;
    int completedCommands;
    
    NCQEntry() {}

    NCQEntry(const DynInstPtr& inst) :
        inst(inst),
        canWB(false),
        completedCommands(0) {}

    // all commands have been finished
    bool finished() const {
        return completedCommands == commands.size();
    }
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

        NCQ* ncq;

        NodeCommandsOrdering ncOrder;

        std::unordered_map<PacketId, NodeCommandPtr> packetIssuers;

    public:
        typedef typename CircularQueue<NCQEntry>::iterator NCQIterator;
        typedef typename std::vector<NodeCommandPtr>::iterator NodeCommandIterator;

        NCQUnit(ThreadID thread_id, int queue_size, NCQ* ncq);
        //NCQUnit(const NCQUnit&) = delete;
        Fault pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd);
        void insertInstruction(const DynInstPtr& inst);
        // commit all instructions before specified seq number
        void commitBefore(InstSeqNum seq_num);
        void writebackCommands();
        void completeCommand(NCQIterator cmd_it);
        void tick();
        bool isFull();
    
        bool handleCacheResp(PacketPtr pkt);
};

}
}
}

#endif

