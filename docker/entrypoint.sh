#!/bin/bash

SCRIPT_PATH="$1"

cd $GITHUB_WORKSPACE/

if ![ -x "$SCRIPT_PATH" ]; then
    echo "You need to specify the path to the script to execute!"
    exit 1
fi

# without further ado, let's start the real work
"$SCRIPT_PATH"

