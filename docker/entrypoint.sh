#!/bin/bash

SCRIPT_PATH="$1"

cd $GITHUB_WORKSPACE/

if ! [ -x "$SCRIPT_PATH" ]; then
    echo "You need to specify the path to the script to execute!"
    exit 1
fi

# a hack to allow files to be removed by runner later
umask o=rwx

# without further ado, let's start the real work
PATH=$PATH:$RISCV_TOOLCHAIN/bin "$SCRIPT_PATH" ${@:2}

