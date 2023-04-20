/*
 * Copyright (c) 2023 National University of Singapore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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

    inline std::string toString() const {
        switch(type) {
            case CAP_TRACK_MEM:
                return std::string("(") + std::to_string(pos.mem.addr) + ")";
            case CAP_TRACK_REG:
                return std::string("(") + std::to_string(pos.reg.threadId) + ", "
                    + std::to_string(pos.reg.regId) + ")";
            default:
                return "";
        }
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

