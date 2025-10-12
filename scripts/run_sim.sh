set -e
cd "$(dirname "$0")/.."
echo "[run_sim] Project dir: $(pwd)"
export GAZEBO_MODEL_PATH="$(pwd)/sim/gazebo/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$(pwd)/build:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}"
if [ ! -f build/libavoider_plugin.so ]; then
  echo "[run_sim] Configuring & building..."
  cmake -B build -S . -DBUILD_GAZEBO_PLUGIN=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  cmake --build build -j
fi
killall -q gzserver gzclient 2>/dev/null || true

WORLD_FILE="$(pwd)/sim/gazebo/pro_world.world"
if [ ! -f "$WORLD_FILE" ]; then
  echo "[run_sim] World not found: $WORLD_FILE"
  exit 1
fi

echo "[run_sim] Starting gzserver..."
gzserver --verbose "$WORLD_FILE" &
SERVER_PID=$!

# انتظر شوية لحد السيرفر ما يقوم
sleep 2

echo "[run_sim] Starting gzclient..."
gzclient --verbose

echo "[run_sim] Shutting down..."
kill $SERVER_PID 2>/dev/null || true
