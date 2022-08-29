from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class SimpleCache(ClockedObject):
    type = 'SimpleCache'
    cxx_header = 'learning_gem5/cache/simple_cache.hh'
    cxx_class = 'gem5::SimpleCache'

    cpu_side = VectorSlavePort('CPU side ports')
    mem_side = MasterPort('Memory side port')

    latency = Param.Cycles(1, 'latency of cache miss')

    size = Param.MemorySize('32kB', 'size of the cache')

    system = Param.System(Parent.any, 'system the cache belongs to')

