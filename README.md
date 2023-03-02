## GEM5 model of Capstone

Please make sure that you have Docker installed on your system.

Build GEM5:
```
./run-docker build
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