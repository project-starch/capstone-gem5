# NOTE: this only works with the RISC-V Capstone arch


import sys
import m5
from m5.objects import *
system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = 'timing'
system.mem_ranges = [AddrRange('512MB')]
system.cpu = TimingSimpleCPU()
system.node_controller = NodeController()
system.cpu.node_controller = system.node_controller

system.membus = SystemXBar()

system.cache = SimpleCache()

system.cpu.icache_port = system.cache.cpu_side
system.cpu.dcache_port = system.cache.cpu_side
system.cpu.ncache_port = system.node_controller.cpu_side

system.cache.mem_side = system.membus.cpu_side_ports

# system.cpu.icache_port = system.membus.cpu_side_ports
# system.cpu.dcache_port = system.membus.cpu_side_ports

system.cpu.createInterruptController()
# system.cpu.interrupts[0].pio = system.membus.mem_side_ports
# system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
# system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

if len(sys.argv) < 2:
    binary = 'tests/capstone/hello'
    args = []
else:
    binary = 'tests/capstone/' + sys.argv[1]
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

