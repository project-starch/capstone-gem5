#ifndef _TYPING_HH_
#define _TYPING_HH_

#include "sim/port.hh"
#include "cpu/simple/base.hh"
#include "params/BaseSimpleCPU.hh"

namespace gem5::RiscvcapstoneISA {

class BaseSimpleCPUWithNodePort : public BaseSimpleCPU {
    public:
        BaseSimpleCPUWithNodePort(const BaseSimpleCPUParams& p): BaseSimpleCPU(p) {}
        virtual Port& getNodePort() = 0;
};

}


#endif
