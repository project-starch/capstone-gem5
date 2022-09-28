from m5.params import *

from m5.objects.BaseSimpleCPU import BaseSimpleCPU

from .NodeController import NodeController

class BaseAtomicSimpleNCacheCPU(BaseSimpleCPU):
    type = 'BaseAtomicSimpleNCacheCPU'
    cxx_header = 'arch/riscvcapstone/atomic_ncache_cpu.hh'
    cxx_class = 'gem5::RiscvcapstoneISA::AtomicSimpleNCacheCPU'

    width = Param.Int(1, "CPU width")
    simulate_data_stalls = Param.Bool(False, "Simulate dcache stall cycles")
    simulate_inst_stalls = Param.Bool(False, "Simulate icache stall cycles")


    ncache_port = RequestPort('node cache port')
    node_controller = Param.NodeController('node controller for revocation nodes')

    @classmethod
    def memory_mode(cls):
        return 'atomic'

    @classmethod
    def support_take_over(cls):
        return True

    def addSimPointProbe(self, interval):
        simpoint = SimPoint()
        simpoint.interval = interval
        self.probeListener = simpoint

