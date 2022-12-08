#include "arch/riscvcapstone/o3/node_commands.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


PacketPtr
NodeAllocate::transition() {
    return nullptr;
}

void
NodeAllocate::handleResp(PacketPtr pkt) {
}


PacketPtr
NodeRevoke::transition() {
    return nullptr;
}

void
NodeRevoke::handleResp(PacketPtr pkt) {
}


PacketPtr
NodeRcUpdate::transition() {
    return nullptr;
}

void
NodeRcUpdate::handleResp(PacketPtr pkt) {
}


PacketPtr
NodeQuery::transition() {
    return nullptr;
}

void
NodeQuery::handleResp(PacketPtr pkt) {
}


}
}
}
