# NOTE: this only works with the RISC-V Capstone arch
# usage: gem5 fast-forward.py ff_n emu_n command
# fast forward ff_n instructions and then emulate emu_n instructions


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

system.mem_mode = 'atomic'
system.mem_ranges = [AddrRange(0x0, size='512MB')]



system.cpu = AtomicSimpleCPU()

system.l2cache = L2Cache()
system.l2bus = L2XBar()


system.membus = SystemXBar()

system.cpu.icache = L1ICache()
system.cpu.dcache = L1DCache()

system.l2bus.mem_side_ports = system.l2cache.cpu_side
system.l2cache.mem_side = system.membus.cpu_side_ports
system.cpu.icache.mem_side = system.l2bus.cpu_side_ports
system.cpu.dcache.mem_side = system.l2bus.cpu_side_ports

system.cpu.icache_port = system.cpu.icache.cpu_side
system.cpu.dcache_port = system.cpu.dcache.cpu_side

system.cpu.mmu.itb.walker.port = system.membus.cpu_side_ports
system.cpu.mmu.dtb.walker.port = system.membus.cpu_side_ports

system.cpu.createInterruptController()

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports


if len(sys.argv) < 4:
    sys.stderr.write('Usage: fast-forward.py <ff_n> <emu_n> <cmd> [cmd-args]')
    sys.exit(1)
else:
    ff_n = int(sys.argv[1])
    emu_n = int(sys.argv[2])
    binary = sys.argv[3]
    args = sys.argv[4:]

system.workload = SEWorkload.init_compatible(binary)

process = Process()
process.cmd = [binary] + args
system.cpu.workload = process

system.cpu.cpu_id = 0
system.cpu.createThreads()

system.cpu.max_insts_any_thread = ff_n

root = Root(full_system = False, system = system)

switchedout_cpu = TimingSimpleCPU(switched_out=True)
switchedout_cpu.system = system
switchedout_cpu.clk_domain = system.cpu.clk_domain
switchedout_cpu.isa = system.cpu.isa
switchedout_cpu.workload = system.cpu.workload
switchedout_cpu.max_insts_any_thread = emu_n
switchedout_cpu.progress_interval = system.cpu.progress_interval
switchedout_cpu.cpu_id = system.cpu.cpu_id
switchedout_cpu.createThreads()
system.switch_cpus = [switchedout_cpu]
switch_list = [(system.cpu, switchedout_cpu)]

m5.instantiate()
print("Beginning simulation (fast-forward)!")

exit_event = m5.simulate()
print('Fast-forward: exiting @ tick {} because {}'
      .format(m5.curTick(), exit_event.getCause()))

if exit_event.getCause() == 'exiting with last active thread context':
    print('Cannot proceed to switch CPUs')
    sys.exit(0)
    
m5.stats.reset()

m5.switchCpus(system, switch_list)
# switch core

exit_event = m5.simulate()
print('Simulation exiting @ tick {} because {}'
        .format(m5.curTick(), exit_event.getCause()))

