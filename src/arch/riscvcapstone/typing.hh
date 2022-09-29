#ifndef _TYPING_HH_
#define _TYPING_HH_

#include "sim/port.hh"
#include "cpu/simple/base.hh"
#include "params/BaseSimpleCPU.hh"


namespace gem5::RiscvcapstoneISA {

class NodeController;

class BaseSimpleCPUWithNodeController : public BaseSimpleCPU {
    public:
        BaseSimpleCPUWithNodeController(const BaseSimpleCPUParams& p): BaseSimpleCPU(p) {}
        virtual Port& getNodePort() = 0;
        virtual NodeController* getNodeController() = 0;
};

}


#endif
