if [ $# != 1 ]; then
    echo "Error! Usage:
    run.sh <num_threads> (Number of threads you'd like to build gem5 with)" >&2
    exit 1
fi

export APPTAINERENV_NUM_THREADS=$1

if ! command -v riscv64-unknown-linux-gnu-gcc &> /dev/null; then
    echo "Error! RISC-V GCC not found!" >&2
else
    echo "RISC-V toolchain found"
fi

if ! [ -e container/install-gem5.sif ]; then
    echo "Building container..."
    apptainer build container/install-gem5.sif container/install-gem5.def
fi

echo "Building gem5..."
./container/install-gem5.sif

echo "Running functional tests..."
cd tests/capstone/o3
git submodule update --init --recursive
make run

