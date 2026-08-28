#!/bin/bash
# Split-machine VOLT stack: robot half on the Jetson, console half here.
#
# The Arduino is on the Jetson, so the deadline-bearing nodes (motion
# controller, router, serial bridge) run there. The console, the gamepad and
# the Ignition shadow run on this workstation. They find each other over DDS
# on a shared ROS_DOMAIN_ID.
#
# This script owns BOTH halves and, importantly, owns tearing the remote half
# down. A console that exits while the Jetson keeps streaming frames would
# leave a robot under power with nothing driving it -- so the remote stack is
# started in its own process group, its PGID recorded on the Jetson, and an
# EXIT trap kills that group however this script ends (Ctrl+C, window close,
# error, or normal exit).
#
# Existing single-machine paths are untouched: volt_run_all.py and the
# SIMULATION / PHYSICAL icons still work exactly as before.

set -eo pipefail

WS="${VOLT_WS:-/home/ros2/Documents/volt_ws}"
JETSON_USER="${VOLT_JETSON_USER:-friday}"
JETSON_HOST="${VOLT_JETSON_HOST:-jetson-ros.local}"
JETSON_WS="${VOLT_JETSON_WS:-/home/friday/volt_ws}"
DOMAIN="${VOLT_ROS_DOMAIN_ID:-17}"
PORT="${VOLT_JETSON_PORT:-}"
DRY="${VOLT_DRY:-true}"
GAZEBO="${VOLT_GAZEBO:-true}"
DDS_PROFILE="$HOME/.config/volt/fastdds_no_shm.xml"
REMOTE_DDS="\$HOME/.config/volt/fastdds_no_shm.xml"
PGID_FILE="/tmp/volt_jetson_stack.pgid"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5
          -o ServerAliveCountMax=3)
TARGET="$JETSON_USER@$JETSON_HOST"

say() { echo "[VOLT/jetson] $*"; }
die() { echo "[VOLT/jetson] ERROR: $*" >&2; exit 1; }

remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }

# ------------------------------------------------------------------ checks --
say "workstation $(hostname) -> robot $TARGET  (ROS_DOMAIN_ID=$DOMAIN)"

remote true 2>/dev/null || die \
"cannot reach $TARGET over SSH with key auth.

Check the Jetson is powered and on the same network, then install a key:
    ssh-copy-id $TARGET"

# The workspace has to exist and be built, or the launch below fails with a
# far less obvious message than this one.
remote "test -f '$JETSON_WS/install/setup.bash'" 2>/dev/null || die \
"$JETSON_WS is not built on the Jetson.

    ssh $TARGET
    cd $JETSON_WS && colcon build --packages-select volt_description --symlink-install"

if [ -z "$PORT" ]; then
    PORT="$(remote 'ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1' || true)"
fi
[ -n "$PORT" ] || die \
"no /dev/ttyUSB* or /dev/ttyACM* on the Jetson.

The Arduino is plugged into the JETSON in this mode, not into this PC."

remote "test -r '$PORT' -a -w '$PORT'" 2>/dev/null || die \
"$JETSON_USER cannot read/write $PORT on the Jetson.

    ssh $TARGET 'sudo usermod -aG dialout $JETSON_USER'   # then log out and back in"

say "Arduino on the Jetson at $PORT"
[ "$DRY" = "false" ] \
    && say "LIVE servo bridge -- ARM manually in the console" \
    || say "dry-run bridge -- servo writes are logged on the Jetson, not sent"

# ------------------------------------------------------------- remote half --
stop_remote() {
    local pgid
    pgid="$(remote "cat $PGID_FILE 2>/dev/null" 2>/dev/null || true)"
    if [ -n "$pgid" ]; then
        say "stopping the robot stack on the Jetson (pgid $pgid)"
        # SIGINT first so the bridge can send its HOLD and the firmware
        # disarms cleanly rather than timing out.
        remote "kill -INT -$pgid 2>/dev/null || true" 2>/dev/null || true
        sleep 3
        remote "kill -9 -$pgid 2>/dev/null || true; rm -f $PGID_FILE" 2>/dev/null || true
    fi
}
trap stop_remote EXIT INT TERM

stop_remote   # clear anything a previous run left behind

REMOTE_CMD="
set -e
source /opt/ros/humble/setup.bash
source '$JETSON_WS/install/setup.bash'
export ROS_DOMAIN_ID=$DOMAIN
export FASTRTPS_DEFAULT_PROFILES_FILE=$REMOTE_DDS
export RCUTILS_COLORIZED_OUTPUT=0
cd '$JETSON_WS'
setsid ros2 launch volt_description volt_jetson.launch.py \
    serial_port:='$PORT' dry_run:='$DRY' \
    > /tmp/volt_jetson_stack.log 2>&1 &
echo \$! > /tmp/volt_jetson_stack.pid
sleep 1
ps -o pgid= -p \$(cat /tmp/volt_jetson_stack.pid) | tr -d ' ' > $PGID_FILE
echo started pgid \$(cat $PGID_FILE)
"

say "starting the robot stack on the Jetson"
remote "$REMOTE_CMD" || die "the robot stack failed to start; see /tmp/volt_jetson_stack.log on the Jetson"

# Give the remote nodes a moment to announce themselves before the console
# starts looking for them, so the operator does not meet a red status panel.
sleep 6
if ! remote "kill -0 -\$(cat $PGID_FILE) 2>/dev/null"; then
    say "--- last 25 lines from the Jetson ---"
    remote "tail -25 /tmp/volt_jetson_stack.log" 2>/dev/null || true
    die "the robot stack exited immediately on the Jetson"
fi
say "robot stack up; streaming its log below the console output"
remote "tail -n +1 -f /tmp/volt_jetson_stack.log" 2>/dev/null | sed 's/^/[jetson] /' &

# -------------------------------------------------------------- local half --
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] || die "$WS/install is missing. Build first:
    cd $WS && colcon build --symlink-install"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"
[ -f "$DDS_PROFILE" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"

say "starting the console on this workstation (Ctrl+C stops both halves)"
ros2 launch volt_description volt_operator.launch.py gazebo:="$GAZEBO"
