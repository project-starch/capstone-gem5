# NOTE: this only works with the RISC-V Capstone arch
# usage: gem5 fast-forward.py ff_n emu_n command
# fast forward ff_n instructions and then emulate emu_n instructions


from ctypes import wstring_at
import sys
import argparse
import m5
from m5.objects import *

parser = argparse.ArgumentParser()
parser.add_argument('--skip', type=int, default=0, help='number of instructions to skip through fast-forwarding')
parser.add_argument('--lim', type=int, default=0, help='max number of instructions to simulate (0 for no limit)')
parser.add_argument('--ncache-size', type=str, default='8kB', help='size of the node cache')
parser.add_argument('--checkpoint-period', type=int, default=0, help='interval between checkpoints')
parser.add_argument('--checkpoint-folder', type=str, default='./checkpoints', help='where to store the checkpoints')
parser.add_argument('--cpu', type=str, default='simple', help='CPU model (atomic, simple, o3)')

if '--' not in sys.argv:
    sys.stderr.write('Usage: fast-forward.py [flags] -- <commands>')
    sys.exit(1)

arg_delimiter_idx = sys.argv.index('--')

args = parser.parse_args(sys.argv[1:arg_delimiter_idx])
commands = sys.argv[arg_delimiter_idx + 1:]

start_with_atomic = args.skip > 0 or args.cpu == 'atomic'

binary = commands[0]
arguments = commands[1:]

if args.cpu == 'simple':
    MainCPU, main_timing = (TimingSimpleCPU, 'timing')
elif args.cpu == 'o3':
    MainCPU, main_timing = (O3CPU, 'timing')
else:
    MainCPU, main_timing = (AtomicSimpleCPU, 'atomic')

InitCPU, init_timing = (AtomicSimpleCPU, 'atomic') if args.skip > 0 else (MainCPU, main_timing)

is_capstone = 'NodeController' in globals()


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

class NCache(Cache):
    size = args.ncache_size
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

system.mem_mode = init_timing
system.mem_ranges = [AddrRange(0x0, size='32GiB')]



system.cpu = InitCPU()

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
system.mem_ctrl.dram.device_size = '2048MiB'
system.mem_ctrl.port = system.membus.mem_side_ports

if is_capstone:
    if args.cpu == 'o3':
        system.ncache = NCache()
        system.cpu.ncache_port = system.ncache.cpu_side
        system.ncache.mem_side = system.membus.cpu_side_ports
        #system.node_controller = CapstoneO3NodeController()
    else:
        system.ncache = NCache()
        system.node_controller = NodeController()
        system.cpu.ncache_port = system.node_controller.cpu_side
        system.cpu.node_controller = system.node_controller
        system.node_controller.mem_side = system.ncache.cpu_side
        system.ncache.mem_side = system.membus.cpu_side_ports


system.workload = SEWorkload.init_compatible(binary)

process = Process()
process.cmd = [binary] + arguments
system.cpu.workload = process

system.cpu.cpu_id = 0
system.cpu.createThreads()


if args.skip > 0:
    system.cpu.max_insts_any_thread = args.skip

root = Root(full_system = False, system = system)

if args.skip > 0:
    switchedout_cpu = MainCPU(switched_out=True)
    switchedout_cpu.system = system
    switchedout_cpu.clk_domain = system.cpu.clk_domain
    switchedout_cpu.isa = system.cpu.isa
    switchedout_cpu.workload = system.cpu.workload
    if args.lim > 0:
        switchedout_cpu.max_insts_any_thread = args.lim
    switchedout_cpu.progress_interval = system.cpu.progress_interval
    if is_capstone:
        switchedout_cpu.node_controller = system.node_controller
    switchedout_cpu.cpu_id = system.cpu.cpu_id
    switchedout_cpu.createThreads()
    system.switch_cpus = [switchedout_cpu]
    switch_list = [(system.cpu, switchedout_cpu)]

m5.instantiate()
if args.skip > 0:
    print("Beginning simulation (fast-forward)!")

    exit_event = m5.simulate()
    print('Fast-forward: exiting @ tick {} because {}'
          .format(m5.curTick(), exit_event.getCause()))

    if exit_event.getCause() == 'exiting with last active thread context':
        print('Cannot proceed to switch CPUs')
        sys.exit(0)
        
    m5.stats.reset()

    m5.switchCpus(system, switch_list) # switch core
else:
    print("Beginning simulation!")

exit_event = m5.simulate()
print('Simulation exiting @ tick {} because {}'
        .format(m5.curTick(), exit_event.getCause()))

# m5.checkpoint('m5out/ckp')

