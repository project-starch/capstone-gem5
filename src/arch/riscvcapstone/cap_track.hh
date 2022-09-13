#ifndef CAP_TRACK_H
#define CAP_TRACK_H

#include "arch/riscvcapstone/types.hh"

namespace gem5::RiscvcapstoneISA {

struct CapLocMem {
    uint64_t addr;
};

inline bool operator < (const CapLocMem& a, const CapLocMem& b) {
    return a.addr < b.addr;
}

struct CapLocReg {
    int threadId;
    RegId regId;
};

inline bool operator < (const CapLocReg& a, const CapLocReg& b) {
    if(a.threadId == b.threadId) {
        return a.regId < b.regId;
    }
    return a.threadId < b.threadId;
}

struct CapLoc {
    enum {
        CAP_TRACK_MEM = 0,
        CAP_TRACK_REG = 1,
    } type;
    union {
        CapLocMem mem;
        CapLocReg reg;
    } pos;
};

inline bool operator < (const CapLoc& a, const CapLoc& b) {
    if(a.type != b.type)
        return a.type < b.type;
    if(a.type == CapLoc::CAP_TRACK_MEM)
        return a.pos.mem < b.pos.mem;
    return a.pos.reg < b.pos.reg;
}

typedef std::map<CapLoc, NodeID> CapTrackMap;

}

#endif

