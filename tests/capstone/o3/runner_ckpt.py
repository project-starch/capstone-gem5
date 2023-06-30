import subprocess
import sys
import argparse
import os
import multiprocessing

parser = argparse.ArgumentParser()
parser.add_argument('--stats-folder', type=str, default='.', help='output folder for stats & config file')

arg_delimiter_idx = sys.argv.index('--')

args = parser.parse_args(sys.argv[1:arg_delimiter_idx])
commands = sys.argv[arg_delimiter_idx + 1:]

binary = commands[0]
binary_name = binary.split('.')[0]

output_dir = "m5out/%s" % (args.stats_folder)
CHECK_FOLDER = os.path.isdir(output_dir)

if not CHECK_FOLDER:
    os.makedirs(output_dir)

if not os.path.isdir("m5out/ckp"):
    os.makedirs("m5out/ckp")

cache_sizes = ['8kB', '16kB', '32kB', '64kB', '128kB', '256kB', '512kB', '1MB']
cache_config = ['l2', 'membus']
# cache_sizes = ['8kB']
# cache_config = ['l2']
commands = []

for i in cache_config:
    for j in cache_sizes:
        k = 8192
        while k <= 524288:
            testcase = "%s_%d_%s_%s" % (binary_name, k, i, j)
            sed_command = "sed 's/xx/%d/g' %s > src/capstone/temp_%s.S" % (k, binary, testcase)
            os.system(sed_command)
            
            command1 = "make run-temp_%s GEM5_FLAGS='--stats-file=%s/%s.txt --dump-config=%s/%s.ini' GEM5_CONFIG_FLAGS='--cpu=o3 --tcache-memside=%s --tcache-size=%s --ckpt-save=m5out/ckp/%s' GEM5_TEST_TIMEOUT=0" % (testcase, args.stats_folder, testcase, args.stats_folder, testcase, i, j, testcase)
            command2 = "make run-temp_%s GEM5_FLAGS='--stats-file=%s/%s.txt --dump-config=%s/%s.ini' GEM5_CONFIG_FLAGS='--cpu=o3 --tcache-memside=%s --tcache-size=%s --ckpt=m5out/ckp/%s' GEM5_TEST_TIMEOUT=0" % (testcase, args.stats_folder, testcase, args.stats_folder, testcase, i, j, testcase)
            commands.append((command1, command2))
            k *= 2

def run_subprocess(command_pair):
     for cmd in command_pair:
        subprocess.run(cmd, shell=True)

with multiprocessing.Pool(processes=10) as pool:
        pool.map(run_subprocess, commands)

subprocess.run("rm -rf m5out/ckp", shell=True)
subprocess.run("rm -rf src/capstone/temp*", shell=True)
