#!/usr/bin/env bash
# Compile and run the host unit tests (needs g++).  Usage: bash tests/run_tests.sh
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++11 -Wall -Wextra -O1 test_power_core.cpp -o /tmp/test_power_core
/tmp/test_power_core
