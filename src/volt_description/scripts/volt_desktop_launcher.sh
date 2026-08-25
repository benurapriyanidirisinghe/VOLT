#!/usr/bin/env bash
# VOLT desktop launcher.
#
# Backs the three clickable desktop entries:
#   sim       Ignition + control GUI, servos untouched (dry-run)
#   gui       control GUI only, attached to an already-running stack
#   physical  Ignition + control GUI + live Arduino bridge
#
# Runs a preflight in dialogs, then re-execs itself inside a terminal window so
# the stack's log is visible and Ctrl-C stops it.
set -o pipefail

MODE="${1:-sim}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
RUNNER="$WS/src/volt_description/scripts/volt_run_all.py"

# The ros_nav operator station owns domain 0; VOLT cannot share it.
export ROS_DOMAIN_ID="${VOLT_ROS_DOMAIN_ID:-17}"

# UDP-only DDS. Fast-DDS shared memory rots on this machine after a hard kill,
# and /dev/shm is shared with ros_nav -- staying off SHM avoids both problems
# without touching the other project's transport.
DDS_PROFILE="$HOME/.config/volt/fastdds_no_shm.xml"
ensure_dds_profile() {
    [ -f "$DDS_PROFILE" ] && return 0
    mkdir -p "$(dirname "$DDS_PROFILE")"
    cat > "$DDS_PROFILE" <<'XML_EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_only</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_participant" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_only</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XML_EOF
}

die_dialog() {
    zenity --error --width=460 --title="VOLT" --text="$1" 2>/dev/null
    exit 1
}

find_serial_port() {
    local dev
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$dev" ] && { echo "$dev"; return 0; }
    done
    return 1
}

# VOLT processes only: matched by pattern AND by a working directory inside the
# workspace, so the ros_nav station's own robot_state_publisher / clock bridge
# is never a candidate. The launcher and its terminal are excluded by name.
volt_pids() {
    local pid cwd cmd out=""
    for pid in $(pgrep -f "volt_run_all|volt_description|ign gazebo|robot_state_publisher|parameter_bridge|controller_manager/spawner" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)" || continue
        case "$cmd" in
            *volt_desktop_launcher*|*gnome-terminal*) continue ;;
        esac
        cwd="$(readlink "/proc/$pid/cwd" 2>/dev/null)" || continue
        case "$cwd" in "$WS"*) out="$out $pid" ;; esac
    done
    echo $out
}

sweep_orphans() {
    local pids; pids="$(volt_pids)"
    [ -z "$pids" ] && return 0
    echo "[VOLT] clearing $(echo $pids | wc -w) leftover process(es) from a previous run"
    kill -9 $pids 2>/dev/null
    sleep 2
}

stack_is_live() {
    timeout 12 ros2 node list 2>/dev/null | grep -q "volt_motion_controller"
}

# The VS Code snap exports a GTK/loader environment that kills gnome-terminal
# with a GLIBC_PRIVATE symbol error. A desktop click is clean, but launching
# from a VS Code terminal is not, so scrub it before spawning anything.
snap_scrub_args() {
    local v
    for v in LD_LIBRARY_PATH LD_PRELOAD GTK_EXE_PREFIX GTK_PATH \
             GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
             GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH GCONV_PATH \
             XDG_DATA_HOME; do
        printf -- '-u\n%s\n' "$v"
    done
    if [ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-}" ]; then
        printf 'XDG_CONFIG_DIRS=%s\n' "$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
    fi
}

open_terminal() {
    local title="$1" inner="$2" term
    local -a scrub=()
    mapfile -t scrub < <(snap_scrub_args)
    for term in /usr/bin/gnome-terminal /usr/bin/x-terminal-emulator /usr/bin/xterm; do
        [ -x "$term" ] || continue
        case "$term" in
            *xterm)
                env "${scrub[@]}" "$term" -title "$title" -e bash -c "$inner" && return 0 ;;
            *)
                env "${scrub[@]}" "$term" --title="$title" -- bash -c "$inner" && return 0 ;;
        esac
    done
    # No usable terminal: run detached and say where the log went.
    local log="$HOME/.local/share/volt-${MODE}.log"
    env "${scrub[@]}" setsid bash -c "$inner" > "$log" 2>&1 < /dev/null &
    notify-send "VOLT" "No terminal emulator available. Logging to $log" 2>/dev/null
    return 0
}

# ---------------------------------------------------------------- preflight --
# Dialog phase. Runs before the terminal window opens.
if [ -z "${VOLT_IN_TERM:-}" ]; then
    [ -x "$RUNNER" ] || [ -f "$RUNNER" ] || die_dialog "Cannot find the VOLT runner:\n$RUNNER"
    VOLT_DRY="true"

    if [ "$MODE" = "physical" ]; then
        PORT="$(find_serial_port)" || die_dialog \
"No Arduino found.\n\nNo /dev/ttyUSB* or /dev/ttyACM* device is present. Connect the Nano and try again.\n\nTo drive the simulation instead, use the VOLT Simulation icon."

        CHOICE="$(zenity --question --width=520 --title="VOLT — connect to physical robot" \
            --ok-label="Launch LIVE" --cancel-label="Cancel" \
            --extra-button="Launch dry-run" \
            --text="<b>This launches the live servo bridge on $PORT.</b>

Build and simulation success do <b>not</b> establish that a servo mapping,
direction, trim, or mechanical limit is safe.

Before continuing:
  •  Raise the robot so its feet cannot reach the ground
  •  Keep the servo-power disconnect within reach
  •  Expect to ARM manually — no motion happens until you do

<i>Launch dry-run</i> starts the identical stack with servo writes logged
instead of sent. Nothing reaches the hardware." 2>/dev/null)"
        rc=$?
        # Zenity returns 1 for BOTH Cancel and the extra button; only the extra
        # button also prints its label. Without reading it, Cancel would launch.
        if [ $rc -eq 0 ]; then
            VOLT_DRY="false"
        elif [ "$CHOICE" = "Launch dry-run" ]; then
            VOLT_DRY="true"
        else
            exit 0
        fi
    fi

    TITLE="VOLT — ${MODE}"
    [ "$MODE" = "physical" ] && [ "$VOLT_DRY" = "false" ] && TITLE="VOLT — PHYSICAL (LIVE SERVOS)"
    INNER="VOLT_IN_TERM=1 VOLT_DRY='$VOLT_DRY' '$0' '$MODE'; echo; \
echo '[VOLT] stack exited. Press Enter to close this window.'; read -r"
    open_terminal "$TITLE" "$INNER"
    exit 0
fi

# ------------------------------------------------------------- in terminal --
VOLT_DRY="${VOLT_DRY:-true}"
ensure_dds_profile
export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
else
    echo "[VOLT] ERROR: $WS/install is missing. Build first:"
    echo "         cd $WS && colcon build --symlink-install"
    exit 1
fi

cd "$WS" || exit 1
echo "[VOLT] mode=$MODE  domain=$ROS_DOMAIN_ID  workspace=$WS"

case "$MODE" in
    gui)
        if ! stack_is_live; then
            zenity --error --width=460 --title="VOLT" --text=\
"No running VOLT stack found on domain $ROS_DOMAIN_ID.\n\nThe control GUI attaches to a stack that is already up. Start one with the VOLT Simulation or VOLT Physical Robot icon first." 2>/dev/null
            exit 1
        fi
        echo "[VOLT] attaching control GUI to the running stack"
        exec python3 "$WS/install/volt_description/lib/volt_description/volt_control_gui.py"
        ;;

    sim)
        sweep_orphans
        echo "[VOLT] starting Ignition + control GUI (servos untouched)"
        exec python3 "$RUNNER" \
            --gazebo-gui true --use-hardware false --dry-run true
        ;;

    physical)
        sweep_orphans
        PORT="$(find_serial_port)" || { echo "[VOLT] ERROR: no serial device"; exit 1; }
        if [ "$VOLT_DRY" = "false" ]; then
            echo "[VOLT] LIVE servo bridge on $PORT -- ARM manually in the GUI"
            exec python3 "$RUNNER" --physical --serial-port "$PORT"
        fi
        echo "[VOLT] dry-run bridge on $PORT -- servo writes are logged, not sent"
        exec python3 "$RUNNER" \
            --gazebo-gui true --use-hardware true --dry-run true --serial-port "$PORT"
        ;;

    *)
        echo "[VOLT] unknown mode '$MODE' (expected sim, gui, or physical)"
        exit 2
        ;;
esac
