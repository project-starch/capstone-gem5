#ifndef __CAPSTONE_TAG_CONTROLLER_H__
#define __CAPSTONE_TAG_CONTROLLER_H__

#include<unordered_set>
#include<list>
#include<vector>
#include "base/types.hh"
#include "arch/riscvcapstone/o3/dyn_inst_ptr.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class MockTagController {
    private:
        struct TagEntry {
            DynInstPtr inst;
            Addr addr;
            bool tagSet;
        };

        using TagQueue = std::list<TagEntry>;

        int threadCount;

        std::unordered_set<Addr> taggedAddrs; // only the committed tags
        std::vector<TagQueue> tagQueues; // uncommitted tags

        static bool aligned(Addr addr) {
            return (addr & ((1 << CAPSTONE_NODE_SIZE_SHIFT) - 1)) == 0;
        }

        void commitTag(const TagEntry& tag_entry) {
            assert(aligned(tag_entry.addr));
            if(tag_entry.tagSet) {
                taggedAddrs.insert(tag_entry.addr);
            } else {
                taggedAddrs.erase(tag_entry.addr);
            }
        }

        Addr lastCommitted = 0;

    public:
        MockTagController(int thread_count);

        bool getTag(Addr addr, ThreadID thread_id) const;
        bool getCommittedTag(Addr addr) const;
        void setTag(const DynInstPtr& inst, Addr addr, bool tag, ThreadID thread_id);
        /**
         * Commit instructions before the given sequence number
         * */
        void commitBefore(InstSeqNum seq_num, ThreadID thread_id);
};

using TagController = MockTagController;

}
}
}


#endif
