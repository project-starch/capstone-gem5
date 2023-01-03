#!/bin/bash

DOWNLOAD_URL="https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2022.12.17/riscv64-elf-ubuntu-22.04-nightly-2022.12.17-nightly.tar.gz"

wget $DOWNLOAD_URL
tar xzf riscv64*.tar.gz
rm riscv64*.tar.gz

echo 'export PATH=$PATH:'$(pwd)/riscv/bin >> ~/.bashrc

