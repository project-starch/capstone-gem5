#ifndef __CAPSTONE_TAG_CONTROLLER_H__
#define __CAPSTONE_TAG_CONTROLLER_H__

#include<type_traits>
#include<unordered_set>
#include<unordered_map> 
#include<list>
#include<vector>
#include "base/types.hh"
#include "base/circular_queue.hh"
#include "mem/port.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class CPU;
class IEW;

class BaseTagController {
    protected:
        struct TagOp { // tag changes
            Addr addr;
            bool tagSet;
        };

        struct TagEntry {
            DynInstPtr inst;
            std::list<TagOp> ops;
            bool canWB;

            void clear() {
                inst = DynInstPtr();
            }
        };

        //using TagQueue = CircularQueue<TagEntry>;
        using TagQueue = CircularQueue<TagEntry>;

        int threadCount, queueSize;

        std::unordered_set<Addr> taggedAddrs; // only the committed tags
        std::vector<TagQueue> tagQueues; // uncommitted tags

        static bool aligned(Addr addr) {
            return (addr & ((1 << CAPSTONE_NODE_SIZE_SHIFT) - 1)) == 0;
        }


        BaseTagController(int thread_count, int queue_size);

        virtual bool writebackTagEntry(TagEntry& tag_entry);
        virtual bool writebackTagOp(DynInstPtr& inst, TagOp& tag_op) = 0;
    
    public:
        using TQIterator = typename TagQueue::iterator;

        bool getTag(const DynInstPtr& inst, Addr addr, bool& delayed);
        virtual bool getCommittedTag(const DynInstPtr& inst, Addr addr, bool& delayed) = 0;
        void setTag(const DynInstPtr& inst, Addr addr, bool tag);
        /**
         * Commit instructions before the given sequence number
         * */
        void commitBefore(const InstSeqNum& seq_num, ThreadID thread_id);
        virtual void tick() = 0;
        virtual void squash(const InstSeqNum& seq_num, ThreadID thread_id);
        virtual void writeback();
        // insert instruction during dispatch (in-order)
        virtual void insertInstruction(const DynInstPtr& inst);

        bool isFull(ThreadID thread_id) {
            assert(thread_id >= 0 && thread_id < threadCount);
            return tagQueues[thread_id].full();
        }
};

class MockTagController : public BaseTagController {
    protected:
        bool writebackTagOp(DynInstPtr& inst, TagOp& tag_op) override;
    public:
        MockTagController(int thread_count, int queue_size);
        MockTagController(const MockTagController& other) = delete;
        bool getCommittedTag(const DynInstPtr& inst, Addr addr, bool& delayed) override;

        void tick() override {}
};

/**
 * Tag controller that uses the actual memory as the subsystem
 * */
class MemoryTagController : public BaseTagController {
    private:
        // for now we use the same mechanism as in the mock controller
        // for the per-thread queue.

        class TagCachePort : public RequestPort {
            private:
                MemoryTagController* owner;
                CPU* cpu;

                PacketPtr blockedPacket;
                bool blocked = false;

                int portsUsed = 0;
                int portsCount;

            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;

            public:
                TagCachePort(MemoryTagController* owner, CPU* cpu,
                        int ports_count);

                void trySendPacket(PacketPtr pkt);

                void tick() {
                    portsUsed = 0;
                }

                bool canSend() {
                    return !blocked && portsUsed < portsCount;
                }
        };

        struct TagCacheRequest {
            DynInstPtr inst;
            TagOp op;
            bool isWrite;
        };

        CPU* cpu;
        IEW* iew;
        TagCachePort tcachePort;

        std::unordered_map<PacketId, TagCacheRequest> ongoingRequests;
            
        Addr getTagAddr(Addr addr) const {
            assert(aligned(addr));
            //return BASE_ADDRESS + (node_id >> (CAPSTONE_NODE_SIZE_SHIFT + 3));
            // each tag takes one bit only
            return BASE_ADDRESS + (addr >> CAPSTONE_NODE_SIZE_SHIFT);
            // TODO: let's be wasteful now and store each tag using a byte
        }

        void trySendPacket(PacketPtr pkt);

    protected:
        bool writebackTagOp(DynInstPtr& inst, TagOp& tag_op) override;

    public:
        const Addr BASE_ADDRESS = 0x70000000; // TODO: for now a random address;
                                              // make this configurable

        MemoryTagController(CPU* cpu, IEW* iew, int thread_count,
                int tcache_ports_count, int queue_size);

        MemoryTagController(const MemoryTagController& other) = delete;

        bool getCommittedTag(const DynInstPtr& inst, Addr addr, bool& delayed) override;
        void tick() override;

        RequestPort& getTagPort() {
            return tcachePort;
        }

        bool handleResp(PacketPtr pkt);
};


#if(false)
#define _MEMBER_CHECK(m) static_assert(std::is_member_function_pointer_v<decltype(&MockTagController::m)> && \
        std::is_member_function_pointer_v<decltype(&MemoryTagController::m)>, "Tag controller member function missing!")

_MEMBER_CHECK(getTag);
_MEMBER_CHECK(getCommittedTag);
_MEMBER_CHECK(setTag);
_MEMBER_CHECK(commitBefore);

#undef _MEMBER_CHECK
#endif

// TODO: make this configurable through configuration scripts.
// This would require dynamic dispatch
#ifdef CAPSTONE_USE_MOCKTAG
using TagController = MockTagController;
#else
using TagController = MemoryTagController;
#endif

}
}
}


#endif
