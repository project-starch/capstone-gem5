#ifndef __CAPSTONE_NODE_H_
#define __CAPSTONE_NODE_H_

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


/*
    revocation node structure
    State (2 bits)
    Counter (33 bit)
    Previous node (31 bits)
    Next node (31 bits)
    Depth (31 bits)
*/
struct Node {
    NodeID prev: 31;
    NodeID next: 31;
    unsigned char state: 2;
    unsigned long long counter: 33;
    unsigned int depth: 31;
};

}
}
}

#endif

