#include "arch/riscvcapstone/o3/ncq.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"



namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

void 
NCQ::insertInstruction(DynInstPtr inst) {
    NodeCommandPtr cmd = inst->getNodeCommand();
    if(cmd != nullptr){
        cmd->inst = inst;
        
        cmdQueue.advance_tail();
        cmdQueue.back() = cmd;
    }
}

void
NCQ::tick() {
}


}
}
}

