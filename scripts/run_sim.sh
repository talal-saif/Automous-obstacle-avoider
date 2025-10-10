#!/bin/bash
# تشغيل وضع المحاكاة (Simulation Mode)

set -e

if [ ! -d build ]; then
  echo "[INFO] Build folder not found. Building project..."
  cmake -B build -S .
  cmake --build build -j
fi

echo "[RUN] Starting simulation mode..."
./build/sim_main
            