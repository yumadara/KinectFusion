#!/bin/bash
# Source it with command: source aliases.sh.

# Get this directory
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build project
alias build="cmake -S ${THIS_DIR} -B ${THIS_DIR}/build"

# Compile project
alias compile="cmake --build ${THIS_DIR}/build"

# Build and compile project
alias build_compile="cmake -S . -B ${THIS_DIR}/build && cmake --build ${THIS_DIR}/build"

# Execute tests
alias test="ctest --test-dir ${THIS_DIR}/build --output-on-failure"

