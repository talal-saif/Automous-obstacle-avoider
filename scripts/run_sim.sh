#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -B build -S .
cmake --build build -j
./build/sim/sim_main
