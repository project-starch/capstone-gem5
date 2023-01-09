#ifndef __CAPSTONE_NODE_H_
#define __CAPSTONE_NODE_H_

#include "arch/riscvcapstone/types.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

const size_t CAPSTONE_NODE_SIZE = 128;

/*
    revocation node structure
    State (2 bits)
    Counter (33 bit)
    Previous node (31 bits)
    Next node (31 bits)
    Depth (31 bits)
*/
struct Node {
    typedef enum {
        FREED = 0,
        VALID = 1,
        INVALID = 2,
        RESERVED = 3,
    } NodeState;

    NodeID prev: 31;
    NodeID next: 31;
    NodeState state: 2;
    unsigned long long counter: 33;
    unsigned int depth: 31;

    bool isValid() {
        return state == VALID;
    }

    void invalidate() {
        state = INVALID;
    }

    void free() {
        assert(state == INVALID); // only invalid nodes can be freed
        state = FREED;
    }
};

static_assert(sizeof(Node) == (CAPSTONE_NODE_SIZE >> 3));

}
}
}

#endif

