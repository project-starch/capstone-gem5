#ifndef __CAPSTONE_CAP_H_
#define __CAPSTONE_CAP_H_

#include<cstdio>
#include<cstddef>
#include<cstdint>
#include "arch/riscvcapstone/types.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

const size_t CAPSTONE_CAP_SIZE = 128; // in bits

/**
 * Representation of a Capstone capability
 * TODO: can consider defining a separate more ergonomic representation and only
 * translate into this when storing it in memory
 * */
struct Cap {
    typedef enum {
        NA = 0, // no access
        RO = 1,
        RW = 2,
        RWX = 3
    } CapPerm;

    typedef enum {
        LIN = 0,
        NONLIN = 1,
        REV = 2,
        UNINIT = 3,
        SEALED = 4,
        SEALEDRET = 5
    } CapType;

    uint64_t cursor;
    unsigned int compressedBound: 27;
    CapPerm perm: 3;
    CapType type: 3;
    NodeID nodeId: 31;
} __attribute__((packed));


static_assert(sizeof(Cap) == (CAPSTONE_CAP_SIZE >> 3));

}
}
}

#endif

