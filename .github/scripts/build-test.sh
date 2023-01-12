#!/bin/bash

print_prompt() {
    echo "#################### "$1" ####################"
}

CURDIR="$(pwd)"
TEST_DIR="tests/capstone/o3"

NTHREADS=$(nproc)
NTHREADS_TO_USE=$(expr $NTHREADS / 4)
if [ "$NTHREADS_TO_USE" -eq 0 ]; then
    NTHREADS_TO_USE=1
fi

GEM5=$(pwd)/build/RISCVCapstone/gem5.opt
GEM5_CONFIG=$(pwd)/configs/capstone/fast-forward.py


build_and_run() {
    # release build
    cd "$CURDIR"

    print_prompt "Starting Release Build"
    if scons build/RISCVCapstone/gem5.opt -j $NTHREADS_TO_USE --linker=mold $BUILD_FLAGS; then
        print_prompt "Release Build Complete"
    else
        print_prompt "Release Build Failed"
        exit 1
    fi



    cd $TEST_DIR
    print_prompt "Building Tests"
    if make; then
        print_prompt "Building Tests Complete"
    else
        print_prompt "Building Tests Failed"
        exit 1
    fi

    print_prompt "Running Tests"
    if make run GEM5="$GEM5" GEM5_CONFIG="$GEM5_CONFIG" \
        GEM5_CONFIG_FLAGS="$GEM5_CONFIG_FLAGS"; then
        print_prompt "Tests Complete"
    else
        print_prompt "Tests Failed"
        exit 1
    fi

    cd "$CUR_DIR"
}

print_prompt "Build and run with the mock tag controller"

BUILD_FLAGS="--uncompressed --mocktag"
GEM5_CONFIG_FLAGS="--cpu=o3 --mocktag"
build_and_run

print_prompt "Build and run with the memory-backed tag controller"

BUILD_FLAGS="--uncompressed"
GEM5_CONFIG_FLAGS="--cpu=o3"
build_and_run


