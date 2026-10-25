#!/bin/bash
# Start the whole Sphere-Scout stack from one terminal.
#
#   tools/run_all.sh [--headless] [--rviz] [--no-bridge] [--no-teleop]
#
# Starts, in order (logs in log/run_all/<name>.log):
#   model      tools/run_model_server.sh                 human-detection service, 127.0.0.1:8766
#   sim        gazebo_world.launch.py                     Gazebo, robot, bridge, EKF
#   control    motor_state_machine                        (use_sim_time)
#   perception perception.launch.py                       celsius node + thermal_detector_node
#   operator   operator.launch.py                         localizer + thermal viewer (RViz with --rviz)
#   viewer     tools/xr_viewer/bridge.py                  browser 3D view, http://localhost:8765
# then runs teleop_twist_keyboard in THIS terminal (keys go to /cmd_vel_teleop), unless --no-teleop.
# Ctrl+C (or leaving teleop) stops everything this script started, and only that: processes are
# tracked by PID, never by name, so other demos on the machine are left alone.
#
#   --headless    Gazebo without a GUI
#   --rviz        also open RViz
#   --no-bridge   skip the browser view
#   --no-teleop   no keyboard; the script just waits (Ctrl+C to stop)
# Environment: ROS_DOMAIN_ID and GZ_PARTITION are inherited (use them to isolate a test run);
#   MODEL_PORT must stay 8766 unless server_url in src/sphere_scout_ai/config/thermal_detector.yaml
#   is changed; BRIDGE_PORT (default 8765) is the browser view's port.
# Every service started here binds 127.0.0.1 and has no authentication.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="$ROOT/log/run_all"
MODEL_PORT="${MODEL_PORT:-8766}"
BRIDGE_PORT="${BRIDGE_PORT:-8765}"
HEADLESS=false; RVIZ=false; BRIDGE=true; TELEOP=true
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=true ;;
    --rviz) RVIZ=true ;;
    --no-bridge) BRIDGE=false ;;
    --no-teleop) TELEOP=false ;;
    -h|--help) sed -n 2,26p "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found at /opt/ros/jazzy" >&2; exit 1; }
[ -f "$ROOT/install/setup.bash" ] || { echo "workspace not built: run  colcon build --symlink-install" >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
mkdir -p "$LOGDIR"

PIDS=(); NAMES=()
start() {                       # start NAME CMD...   (own session, so the whole group can be stopped)
  local name="$1"; shift
  setsid "$@" > "$LOGDIR/$name.log" 2>&1 < /dev/null &   # stdin closed: only teleop may read the keys
  PIDS+=("$!"); NAMES+=("$name")
  echo "  started $name (pid $!)  log: log/run_all/$name.log"
}
alive() { kill -0 "$1" 2>/dev/null; }
cleanup() {
  trap - EXIT INT TERM
  echo; echo "stopping..."
  local i
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -INT -- "-${PIDS[$i]}" 2>/dev/null; done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    local any=false
    for pid in "${PIDS[@]}"; do alive "$pid" && any=true; done
    $any || break
    sleep 1
  done
  for pid in "${PIDS[@]}"; do alive "$pid" && kill -TERM -- "-$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]}"; do alive "$pid" && kill -KILL -- "-$pid" 2>/dev/null; done
  echo "stopped."
}
trap cleanup EXIT INT TERM

fail() { echo "ERROR: $1" >&2; echo "--- last lines of log/run_all/$2.log:" >&2; tail -n 15 "$LOGDIR/$2.log" >&2; exit 1; }

echo "starting Sphere-Scout (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
start model "$ROOT/tools/run_model_server.sh" "$MODEL_PORT"
start sim ros2 launch sphere_scout_bringup gazebo_world.launch.py "headless:=$HEADLESS" rviz:=false

echo "waiting for the model service..."
for i in $(seq 1 60); do
  curl -s -o /dev/null "http://127.0.0.1:$MODEL_PORT/health" && break
  alive "${PIDS[0]}" || fail "the model service exited" model
  sleep 1
  [ "$i" = 60 ] && fail "the model service did not answer within 60 s" model
done
echo "waiting for Gazebo (/clock)..."
for i in $(seq 1 90); do
  timeout 3 ros2 topic echo /clock --once > /dev/null 2>&1 && break
  alive "${PIDS[1]}" || fail "the simulation exited" sim
  [ "$i" = 90 ] && fail "no /clock within 90 s" sim
done

start control ros2 run sphere_scout_control motor_state_machine --ros-args -p use_sim_time:=true
start perception ros2 launch sphere_scout_ai perception.launch.py
start operator ros2 launch sphere_scout_bringup operator.launch.py "rviz:=$RVIZ"
if $BRIDGE; then start viewer python3 "$ROOT/tools/xr_viewer/bridge.py" --port "$BRIDGE_PORT"; fi

echo
echo "up. browser view: http://localhost:$BRIDGE_PORT   scan: ros2 service call /mission/scan std_srvs/srv/Trigger"
if $TELEOP; then
  echo "teleop: keys below drive the robot; Ctrl+C stops everything."
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
else
  echo "Ctrl+C stops everything."
  while alive "${PIDS[1]}"; do sleep 2; done
  echo "the simulation exited."
fi
