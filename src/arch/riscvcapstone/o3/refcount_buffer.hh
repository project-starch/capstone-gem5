#ifndef __CAPSTONE_REFCOUNT_BUFFER_H_
#define __CAPSTONE_REFCOUNT_BUFFER_H_

#include <unordered_map>
#include "arch/riscvcapstone/types.hh"

namespace gem5 {
    namespace RiscvcapstoneISA {
        namespace o3 {


struct RefCountBufEntry {
    NodeID node;
    uint64_t rcDelta;
};

class RefCountBuf {
    private:
        int bufSize;
        bool blocked;

        RefCountBufEntry skidEntry;
        std::unordered_map<NodeID, RefCountBufEntry> rcBuf;

    protected:
        void writeback();

    public:
        RefCountBuf(int buf_size) : bufSize(buf_size), blocked(false) {}
        
        bool addUpdate(NodeID node, uint64_t delta);
        bool containsNode(NodeID node) {
            return rcBuf.find(node) != rcBuf.end();
        }

        void tick();

        bool isBlocked() { return blocked; }
};
        } // end of namespace o3
    } // end of namespace RiscvcapstoneISA
} // end of namespace gem5


#endif

