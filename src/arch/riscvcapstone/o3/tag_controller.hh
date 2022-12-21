#ifndef __CAPSTONE_TAG_CONTROLLER_H__
#define __CAPSTONE_TAG_CONTROLLER_H__

#include<unordered_set>
#include<list>
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

        std::unordered_set<Addr> taggedAddrs; // only the committed tags
        std::list<TagEntry> tagQueue; // uncommitted tags

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
        bool getTag(Addr addr) const;
        bool getCommittedTag(Addr addr) const;
        void setTag(const DynInstPtr& inst, Addr addr, bool tag);
        /**
         * Commit instructions before the given sequence number
         * */
        void commit(InstSeqNum seq_num);
};

using TagController = MockTagController;

}
}
}


#endif
