# CAPStone gem5 simulator

This repo contains an out of order CPU model for CAPStone, built on top
of gem5's O3 CPU model. This design is intended to be used for performance evaluations.

This version of the CAPStone implementation is based on the CAPStone Academic Spec ([commit](https://github.com/project-starch/capstone-academic-spec/commit/ac7f329100489f843770d815ee14ce2b90215ddb))

## Dependencies
To build, make sure you have apptainer installed.

To build the testcases, please make sure you have the RISC-V glibc GCC [toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)
installed and added to `PATH`.

## Running

Simply run the following from the project root directory:

```
chmod +x run.sh
./run.sh <NUM_THREADS>
```

`NUM_THREADS` is the number of threads you'd like to build gem5 with.

This will automatically build an apptainer image with all dependencies
required to build gem5, and use that image to build gem5 and run Capstone
functional tests.

If you make any changes to the source, you can simply rebuild and check it by running `run.sh`. The major portion of the CAPStone related code is in the `src/arch/riscvcapstone` directory.

To run a single testcase, from `tests/capstone/o3` directory:

```
make run-<testcase>
```

## Contributions

Contributions are welcome! Feel free to submit issues or pull requests.
