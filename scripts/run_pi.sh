#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if ! systemctl is-active --quiet pigpiod; then
  echo "Starting pigpio daemon..."
  sudo systemctl start pigpiod
fi

if [ ! -x "$ROOT/build/pi/aoa_pi" ]; then
  cmake -S "$ROOT/pi" -B "$ROOT/build/pi"
  cmake --build "$ROOT/build/pi" -j
fi

exec "$ROOT/build/pi/aoa_pi"
