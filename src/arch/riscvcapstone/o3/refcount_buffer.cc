#include "base/types.hh"
#include "arch/riscvcapstone/o3/refcount_buffer.hh"


namespace gem5 {
    namespace RiscvcapstoneISA {
        namespace o3 {
        
bool
RefCountBuf::addUpdate(NodeID node, uint64_t delta) {
    assert(!blocked);
    if(rcBuf.find(node) != rcBuf.end()) {
        rcBuf[node].rcDelta += delta;
    } else {
        RefCountBufEntry new_entry;
        new_entry.node = node;
        new_entry.rcDelta = delta;
        if(rcBuf.size() < bufSize) {
            rcBuf[node] = new_entry;
        } else {
            blocked = true;
            skidEntry = new_entry;
            return true;
        }
    }
    return false;
}

void
RefCountBuf::tick() {
    writeback();
}

void
RefCountBuf::writeback() {
    bool written_back = false;

    // TODO: writeback whatever there is

    if(written_back) {
        assert(rcBuf.size() < bufSize);
        if(blocked) {
            assert(rcBuf.find(skidEntry.node) == rcBuf.end());
            rcBuf[skidEntry.node] = skidEntry;
            blocked = false; // TODO: can receive one request in this cycle
        }
    }
}

        } // end of namespace o3
    } // end of namespace RiscvcapstoneISA
} // end of namespace gem5
