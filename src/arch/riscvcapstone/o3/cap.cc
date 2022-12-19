#include "arch/riscvcapstone/o3/cap.hh"



namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


uint64_t
CompressedCap::start() const {
    return 0;
}

uint64_t
CompressedCap::end() const {
    return 0;
}

CompressedCap&
CompressedCap::setAddresses(uint64_t start, uint64_t end, uint64_t cursor) {
    return *this;
}

}
}
}
