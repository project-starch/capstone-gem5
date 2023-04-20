## GEM5 model for Capstone evaluation

_Note that this is not a fully implemented GEM5 model for Capstone, but one that models the revocation tree operations only for an early performance evaluation._

Please make sure that you have Docker (version 20 or above recommended)
installed on your system. If you have pulled and cached `corank/gem5-dev` on your
system before, please make sure it is the latest with
```
docker pull corank/gem5-dev
```

Alternatively, if you prefer to build the Docker image:
```
cd docker-build
docker build . -t corank/gem5-dev
```

Build GEM5:
```
./run-docker build
```

Run a minimal test case to check whether GEM5 is successfully built:
```
./run-docker run-hello
```
The output should include something like
```
Beginning simulation!
build/RISCVCapstone/sim/simulate.cc:194: info: Entering event queue @ 0.  Starting simulation...
build/RISCVCapstone/sim/syscall_emul.hh:1020: warn: readlink() called on '/proc/self/exe' may yield unexpected results in various settings.
      Returning '/workspace/tests/capstone/hello'
build/RISCVCapstone/sim/syscall_emul.cc:74: warn: ignoring syscall mprotect(...)
Hello gem5!
Simulation exiting @ tick 59197000 because exiting with last active thread context
```

To build SPEC, make sure that you have placed SPEC 2017 (not included in this repository) in a folder
named `spec` at the root of the repository directory (note that the content should be directly placed in
`spec` rather than inside a subfolder), then apply a patch
```
(cd spec && patch -p1 < ../tests/capstone/speckle/spec17.patch)
```
and
```
./run-docker build-spec
```

To run the SPEC 2017 intspeed ref workloads with the Capstone model:
```
./run-docker run-capstone --multiproc
```

To run the SPEC 2017 intspeed ref workloads with the baseline RISC-V CPU model:
```
./run-docker run-baseline --multiproc
```

Note that the `--multiproc` is optional and enables running the workloads
in parallel.
Removing it will make the run use only one thread running one workload at
a time.

To run a specific workload, use
```
./run-docker run-capstone <workload-name>
```
or
```
./run-docker run-baseline <workload-name>
```

After running the workloads with *both* the Capstone model and the baseline, collect
the results and output them in the LaTeX table format using
```
./run-docker collect-results
```
