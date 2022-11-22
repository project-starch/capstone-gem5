from m5 import *
from m5.proxy import *
from m5.params import *
from m5.objects.ClockedObject import ClockedObject


class CapstoneO3NodeController(ClockedObject):
    type = 'CapstoneO3NodeController'
    cxx_header = 'arch/riscvcapstone/o3/node_controller.hh'
    cxx_class = 'gem5::RiscvcapstoneISA::o3::NodeController'

    # TODO: switching to vector port to allow multicores
    cpu_side = ResponsePort('port typically connected to the CPU dcache port')
    mem_side = RequestPort('memory-side port')

    system = Param.System(Parent.any, 'the system this node controller belongs to')

