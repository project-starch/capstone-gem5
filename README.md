## GEM5 model of Capstone

Please make sure that you have Docker installed on your system.

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
named `spec` at the root of the repository directory, then apply a patch
```
(cd spec && patch -p1 < ../tests/capstone/speckle/spec17.patch)
```
and
```
./run-docker build-spec
```

To run the SPEC 2017 intspeed ref workloads with the Capstone model:
```
./run-docker run-capstone
```

To run the SPEC 2017 intspeed ref workloads with the baseline RISC-V CPU model:
```
./run-docker run-baseline
```