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

        Port& getNodePort() {
            return ncachePort;
        }

        void cacheUnblocked();

        bool canSend() const {
            return ncachePort.canSend();
        }

        bool trySendPacket(PacketPtr pkt, ThreadID thread_id);
        bool handleCacheResp(PacketPtr pkt);

        bool passedQuery(const DynInstPtr& inst) const;
        void cleanupCommands();

        void squash(const InstSeqNum& squashed_num, ThreadID thread_id);
};

}
}
}

#endif

