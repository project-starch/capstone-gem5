# NOTE: this only works with the RISC-V Capstone arch


import sys
import m5
from m5.objects import *


class L1Cache(Cache):
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

class L1ICache(L1Cache):
    size = '16kB'

class L1DCache(L1Cache):
    size = '64kB'

class NCache(NoncoherentCache):
    size = '8kB'
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

class L2Cache(Cache):
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(0x0, size='512MB'), AddrRange(0x100000000000, size='64MB')]
system.cpu = TimingSimpleCPU()
system.node_controller = NodeController()
system.cpu.node_controller = system.node_controller

system.node_mem_ctrl = MemCtrl()
system.node_mem_ctrl.dram = DDR3_1600_8x8()
system.node_mem_ctrl.dram.range = system.mem_ranges[1] 

system.l2cache = L2Cache()
system.l2bus = L2XBar()


system.membus = SystemXBar()

system.ncache = NCache()

system.cpu.icache = L1ICache()
system.cpu.dcache = L1DCache()

system.l2bus.mem_side_ports = system.l2cache.cpu_side
system.l2cache.mem_side = system.membus.cpu_side_ports
system.cpu.icache.mem_side = system.l2bus.cpu_side_ports
system.cpu.dcache.mem_side = system.l2bus.cpu_side_ports

system.cpu.icache_port = system.cpu.icache.cpu_side
system.cpu.dcache_port = system.cpu.dcache.cpu_side

system.cpu.ncache_port = system.node_controller.cpu_side
system.node_controller.mem_side = system.ncache.cpu_side

system.ncache.mem_side = system.node_mem_ctrl.port


system.cpu.createInterruptController()

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

if len(sys.argv) < 2:
    sys.stderr.write('No executable supplied!\n')
    sys.exit(1)
else:
    binary = sys.argv[1]
    args = sys.argv[2:]

system.workload = SEWorkload.init_compatible(binary)

process = Process()
process.cmd = [binary] + args
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system = False, system = system)
m5.instantiate()
print("Beginning simulation!")
exit_event = m5.simulate()

print('Exiting @ tick {} because {}'
      .format(m5.curTick(), exit_event.getCause()))

