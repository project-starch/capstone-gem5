#ifndef __CAPSTONE_NODE_COMMAND_QUEUE_H_
#define __CAPSTONE_NODE_COMMAND_QUEUE_H_

#include <unordered_map>
#include <vector>
#include <list>
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "mem/port.hh"
#include "arch/riscvcapstone/o3/limits.hh"
#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/ncq_unit.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class CPU;
class IEW;

// TODO: also needs to be one per thread
class NCQ {
    public:

        // Node cache port for the NCQ
        class NcachePort : public RequestPort {
            private:
                NCQ* ncq;
                CPU* cpu;

                int portSize; // number of available ports
                int portUsed;
                
                PacketPtr blockedPacket;
                bool blocked;
            public:
                NcachePort(NCQ* ncq, CPU* cpu,
                        int size);

                void tick();
                bool trySendPacket(PacketPtr pkt);
                bool isBlocked() const { return blocked; }
                bool portAvailable() const { 
                    return portUsed < portSize;
                }
                bool canSend() const {
                    return portAvailable() && !blocked;
                }
            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;
        };


    private:
        CPU* cpu;
        IEW* iew;
        std::vector<NCQUnit> threads;
        int queueSize;
        int threadNum;
        std::list<ThreadID>* activeThreads;

        std::unordered_map<PacketId, ThreadID> packetIssuerThreads;

        NcachePort ncachePort;

    public:
        NCQ(CPU* cpu, IEW* iew, int queue_size, int thread_num);

        /** Insert instruction in the NCQ. The order of insts in the queue
         * follows the program order.
        */
        void insertInstruction(const DynInstPtr& inst);
        void tick();

        bool isFull(ThreadID thread_id);

        /** Add a command to the NCQ's corresponding entry for inst. */
        Fault pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd);

        /** Mark all commands before the given seq_num as being able to be committed. */
        void commitBefore(InstSeqNum seq_num, ThreadID thread_id);

        /** This is where the node commands are actually executed. */
        void writebackCommands();
        void writebackCommands(ThreadID thread_id);
        
        void setActiveThreads(std::list<ThreadID>* active_threads) {
            activeThreads = active_threads;
        }

        Port& getNodePort() {
            return ncachePort;
        }

        void cacheUnblocked();

        bool canSend() const {
            return ncachePort.canSend();
        }

        bool trySendPacket(PacketPtr pkt, ThreadID thread_id);
        bool handleCacheResp(PacketPtr pkt);

        QueryResult passedQuery(const DynInstPtr& inst) const;

        /** Remove NCQEntries with completed commands from the NCQ. */
        void cleanupCommands();

        void squash(const InstSeqNum& squashed_num, ThreadID thread_id);

        Fault postExecCheck(const DynInstPtr& inst);

        void allocateInit(ThreadID thread_id);
};

}
}
}

#endif

