from m5 import *
from m5.objects.BaseTimingSimpleCPU import BaseTimingSimpleCPU


class BaseTimingSimpleNCacheCPU(BaseTimingSimpleCPU):
    type = 'BaseTimingSimpleNCacheCPU'
    cxx_header = 'arch/riscvcapstone/ncache_cpu.hh'
    cxx_class = 'gem5::RiscvcapstoneISA::TimingSimpleNCacheCPU'

    ncache_port = RequestPort('node cache port')


