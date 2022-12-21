#ifndef __CAPSTONE_TAG_CONTROLLER_H__
#define __CAPSTONE_TAG_CONTROLLER_H__

#include<unordered_set>
#include "base/types.hh"
#include "arch/riscvcapstone/o3/node.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

class MockTagController {
    private:
        std::unordered_set<Addr> taggedAddrs;

        static bool aligned(Addr addr) {
            return (addr & ((1 << CAPSTONE_NODE_SIZE_SHIFT) - 1)) == 0;
        }

    public:
        bool getTag(Addr addr) const {
            // check the alignment
            assert(aligned(addr));
            return taggedAddrs.find(addr) != taggedAddrs.end();
        }

        void setTag(Addr addr, bool tag) {
            assert(aligned(addr));
            if(tag) {
                taggedAddrs.insert(addr);
            } else {
                taggedAddrs.erase(addr);
            }
        }
};

using TagController = MockTagController;

}
}
}


#endif
