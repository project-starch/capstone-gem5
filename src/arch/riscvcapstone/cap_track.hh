#ifndef CAP_TRACK_H
#define CAP_TRACK_H

#include "arch/riscvcapstone/types.hh"

namespace gem5::RiscvcapstoneISA {

struct CapLocMem {
    Addr addr;
};

inline bool operator < (const CapLocMem& a, const CapLocMem& b) {
    return a.addr < b.addr;
}

struct CapLocReg {
    int threadId;
    RegIndex regId;
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

    CapLoc() {}

    static CapLoc makeMem(Addr addr) {
        CapLoc res;
        res.type = CAP_TRACK_MEM;
        res.pos.mem.addr = addr;
        return res;
    }

    static CapLoc makeReg(int thread_id, RegIndex reg_id) {
        CapLoc res;
        res.type = CAP_TRACK_REG;
        res.pos.reg.threadId = thread_id;
        res.pos.reg.regId = reg_id;
        return res;
    }
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

