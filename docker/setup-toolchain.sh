#!/bin/bash

DOWNLOAD_URL="https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2023.01.04/riscv64-glibc-ubuntu-22.04-nightly-2023.01.04-nightly.tar.gz"

wget $DOWNLOAD_URL
tar xzf riscv64*.tar.gz
rm riscv64*.tar.gz

echo 'export PATH=$PATH:'$(pwd)/riscv/bin >> ~/.bashrc

