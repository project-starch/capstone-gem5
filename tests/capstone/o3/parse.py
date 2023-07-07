import os
import sys
import glob, csv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--input-folder', type=str, default='', help='input folder to parse files from')

arg_delimiter_idx = sys.argv.index('--')

args = parser.parse_args(sys.argv[1:arg_delimiter_idx])
commands = sys.argv[arg_delimiter_idx + 1:]

binary = commands[0]
n = len(binary)

os.chdir(args.input_folder)

l2dict = {}
memdict = {}

for filename in glob.glob("*.txt"):
    val = filename[n+1:]
    coord = val.split('_')
    tuple1 = (coord[0], coord[2].split('.')[0])

    file1 = open(filename, 'r')
    Lines = file1.readlines()

    tcache_hits = "0"
    tcache_misses = "0"
    dcache_hits = "0"
    dcache_misses = "0"
    ncache_hits = "0"
    ncache_misses = "0"
    cycles = ""

    for line in Lines:
        xyz = line.split()
        if "simTicks" in line:
            if coord[1] == 'l2':
                cycles = "%d, %.2f" % (int(xyz[1])/1000, round(int(xyz[1])/(1000*int(coord[0])),2))
            else:
                cycles = "%d, %.2f" % (int(xyz[1])/1000, round(int(xyz[1])/(1000*int(coord[0])),2))
        elif "tcache.overallHits::total" in line:
            tcache_hits = xyz[1]
        elif "tcache.overallMisses::total" in line:
            tcache_misses = xyz[1]
        elif "dcache.overallHits::total" in line:
            dcache_hits = xyz[1]
        elif "dcache.overallMisses::total" in line:
            dcache_misses = xyz[1]
        elif "ncache.overallHits::total" in line:
            ncache_hits = xyz[1]
        elif "ncache.overallMisses::total" in line:
            ncache_misses = xyz[1]

    if coord[1] == 'l2':
        l2dict[tuple1] = cycles + "\nTcache: "+tcache_hits+", "+tcache_misses + "\nDcache: "+dcache_hits+", "+dcache_misses + "\nNcache: "+ncache_hits+", "+ncache_misses
    else:
        memdict[tuple1] = cycles + "\nTcache: "+tcache_hits+", "+tcache_misses + "\nDcache: "+dcache_hits+", "+dcache_misses + "\nNcache: "+ncache_hits+", "+ncache_misses

    file1.close()

def create_csv_sheet(data, filename):
    rows = sorted(set(key[0] for key in data.keys()), key=lambda x: (len(x), x))
    columns = sorted(set(key[1] for key in data.keys()), key=lambda x: (len(x), x))

    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile, quoting=csv.QUOTE_ALL)

        # Write the column headers
        writer.writerow([""] + columns)

        # Write the data rows
        for row in rows:
            writer.writerow([row] + [data.get((row, col), 0) for col in columns])

create_csv_sheet(l2dict, 'l2.csv')
create_csv_sheet(memdict, 'mem.csv')