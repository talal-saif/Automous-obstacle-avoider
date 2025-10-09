#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$ROOT/sim" -B "$ROOT/build/sim"
cmake --build "$ROOT/build/sim" -j
export GAZEBO_PLUGIN_PATH="$ROOT/build/sim:$GAZEBO_PLUGIN_PATH"
exec gazebo "$ROOT/sim/worlds/aoa.world" --verbose
