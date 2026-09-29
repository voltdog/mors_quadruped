#!/bin/bash

set -euo pipefail

# Keep every launched component in a process group separate from this script.
# Job control must stay disabled so the PID returned for `setsid` is also the
# process-group ID used during cleanup.
set +m

    process_groups=()
    helper_groups=()
    LAST_PGID=""
    cleanup_started=false

    start_component() {
        local name="$1"
        shift

        # Bash makes background commands ignore SIGINT and SIGQUIT. Restore
        # the default signal dispositions before executing the component.
        setsid -- env --default-signal=INT,QUIT,TERM "$@" &
        local pgid=$!

        process_groups+=("$pgid")
        LAST_PGID="$pgid"

        echo "Started ${name}: PGID=${pgid}"
    }

    signal_groups() {
        local signal="$1"
        local pgid

        for pgid in "${process_groups[@]}"; do
            kill "-${signal}" -- "-${pgid}" 2>/dev/null || true
        done
    }

    groups_are_running() {
        local pgid

        for pgid in "${process_groups[@]}"; do
            if kill -0 -- "-${pgid}" 2>/dev/null; then
                return 0
            fi
        done

        return 1
    }

    ensure_robot_state_python_binding() {
        local python_bin="${SCRIPT_DIR}/.mpc_venv/bin/python"
        local lcm_msgs_dir="${SCRIPT_DIR}/lcm_msgs"

        if "$python_bin" -c '
import sys
sys.path.insert(0, sys.argv[1])
from mors_msgs.robot_state_msg import robot_state_msg
assert hasattr(robot_state_msg(), "timestamp")
' "$lcm_msgs_dir" 2>/dev/null; then
            return
        fi

        if ! command -v lcm-gen >/dev/null 2>&1; then
            echo "Python ROBOT_STATE binding is stale and lcm-gen is unavailable" >&2
            exit 1
        fi

        echo "Regenerating Python ROBOT_STATE binding..."
        (
            cd "$lcm_msgs_dir"
            lcm-gen -p --ppath . robot_state_msg.lcm
        )
    }

    cleanup() {
        local status=$?
        local pgid

        if [ "$cleanup_started" = true ]; then
            return
        fi
        cleanup_started=true

        # Do not allow a repeated Ctrl+C to interrupt the grace period before
        # SIGTERM/SIGKILL are sent to components that did not stop on SIGINT.
        trap '' SIGINT SIGTERM
        trap - EXIT
        echo "Stopping robot components..."

        # Stop logger timeout helpers together with their `sleep` children.
        for pgid in "${helper_groups[@]}"; do
            kill -TERM -- "-${pgid}" 2>/dev/null || true
        done

        # Let ROS 2 nodes and hardware components shut down gracefully first.
        signal_groups INT

        if groups_are_running; then
            sleep 2
            signal_groups TERM
        fi

        if groups_are_running; then
            sleep 1
            signal_groups KILL
        fi

        wait 2>/dev/null || true
        exit "$status"
    }

    trap 'exit 130' SIGINT
    trap 'exit 143' SIGTERM
    trap cleanup EXIT

	echo "Launching WBIC Quadruped Controller..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "Script Location: $SCRIPT_DIR"

	config_dir="$SCRIPT_DIR/config/"
	echo "Config Location: ${config_dir}"
    locomotion_config="${config_dir}/locomotion_controller.yaml"
    algorithm="$(
        awk '
            /^[[:space:]]*algorithm[[:space:]]*:/ {
                value = $0
                sub(/^[^:]*:[[:space:]]*/, "", value)
                sub(/[[:space:]]*#.*/, "", value)
                gsub(/"/, "", value)
                gsub(/\047/, "", value)
                gsub(/[[:space:]]/, "", value)
                print value
                exit
            }
        ' "$locomotion_config"
    )"

    case "$algorithm" in
        wbic|vision|dcm)
            echo "Selected Algorithm: ${algorithm}"
            ;;
        "")
            echo "Missing algorithm in ${locomotion_config}" >&2
            exit 1
            ;;
        *)
            echo "Unknown algorithm in ${locomotion_config}: ${algorithm}" >&2
            echo "Supported algorithms: wbic, vision, dcm" >&2
            exit 1
            ;;
    esac

    # Fast DDS shared-memory transport often leaves stale locks in /dev/shm.
    # Force UDP transport for this local bringup to avoid SHM port conflicts.
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

    lcm_fallback_url="${LCM_DEFAULT_URL:-${LCM_LOCOMOTION_URL:-}}"
    if [ -z "$lcm_fallback_url" ]; then
        echo "LCM_DEFAULT_URL or LCM_LOCOMOTION_URL must be set" >&2
        exit 1
    fi
    export LCM_LOCOMOTION_URL="${LCM_LOCOMOTION_URL:-$lcm_fallback_url}"
    export LCM_SERVO_URL="${LCM_SERVO_URL:-$lcm_fallback_url}"

    sim_mode=false
    rviz_enabled=false
    logging_enabled=false

    for arg in "$@"; do
        case "$arg" in
            --sim)
                sim_mode=true
                ;;
            --rviz)
                rviz_enabled=true
                ;;
            --log)
                logging_enabled=true
                ;;
            -h|--help)
                echo "Usage: $0 [--sim] [--rviz] [--log]" >&2
                exit 0
                ;;
            *)
                echo "Unknown option: $arg" >&2
                echo "Usage: $0 [--sim] [--rviz] [--log]" >&2
                exit 1
                ;;
        esac
    done

    if [ "$sim_mode" = true ]; then
        height_map_builder="${SCRIPT_DIR}/HeightMapBuilder/build/height_map_builder"
        if [ ! -x "$height_map_builder" ]; then
            echo "Missing executable: ${height_map_builder}" >&2
            echo "Build it with: cmake -S HeightMapBuilder -B HeightMapBuilder/build && cmake --build HeightMapBuilder/build -j" >&2
            exit 1
        fi

        ensure_robot_state_python_binding
    else
        state_estimator="${SCRIPT_DIR}/StateEstimator/build/state_estimator"
        if [ ! -x "$state_estimator" ]; then
            echo "Missing executable: ${state_estimator}" >&2
            echo "Build it with: cmake -S StateEstimator -B StateEstimator/build && cmake --build StateEstimator/build -j" >&2
            exit 1
        fi
    fi

    # ros2 controller
    start_component \
        "robot mode controller" \
        ros2 launch robot_mode_controller bringup.launch.py

    if [ "$sim_mode" = true ]; then
        echo "-------------- Simulation Mode Activated --------------"
        start_component \
            "MORS simulator" \
            "${SCRIPT_DIR}/.mpc_venv/bin/python" "${SCRIPT_DIR}/Simulator/mors_simulator.py"
        simulator_pgid="$LAST_PGID"

        start_component \
            "height map builder (simulation)" \
            "$height_map_builder" "${config_dir}heightmap_builder.yaml"
        height_map_builder_pgid="$LAST_PGID"

        sleep 2s

        if ! kill -0 -- "-${simulator_pgid}" 2>/dev/null; then
            echo "MORS simulator exited during startup" >&2
            exit 1
        fi
        if ! kill -0 -- "-${height_map_builder_pgid}" 2>/dev/null; then
            echo "HeightMapBuilder exited during startup" >&2
            exit 1
        fi
    else
        # hardware interfaces
        echo "-------------- Hardware Mode Activated --------------"
        start_component \
            "BHI360 IMU" \
            "${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu"
        start_component \
            "contact sensor" \
            "${SCRIPT_DIR}/ContactSensor/build/contact_sensor"
        # in the future add here contact sensors controller
        # wait before the other controllers start to ensure that the hardware interfaces are up and running

        start_component \
            "state estimator" \
            "$state_estimator"

        sleep 2s

        start_component \
            "radiolink control" \
            ros2 run mors_radiolink_control mors_radiolink_control
    fi

    if [ "$rviz_enabled" = true ]; then
        if [ "$algorithm" = dcm ]; then
            rviz_config="$SCRIPT_DIR/ros_ws/src/robot_state_viewer/rviz/rviz_config_dcm.rviz"
        else
            rviz_config="$SCRIPT_DIR/ros_ws/src/robot_state_viewer/rviz/rviz_config.rviz"
        fi
        start_component \
            "robot state viewer" \
            ros2 launch robot_state_viewer robot_state_viewer.launch.py \
            "rviz_config:=${rviz_config}"
        sleep 1s
    fi

	# robot control
    case "$algorithm" in
        wbic)
            start_component \
                "MPC locomotion controller" \
                "${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC"
            ;;
        vision)
            start_component \
                "MPC locomotion controller" \
                "${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC"
            ;;
        dcm)
            start_component \
                "DCM locomotion controller" \
                "${SCRIPT_DIR}/LocomotionControllerDCM/build/locomotionControllerDCM"
            ;;
    esac

    echo "Robot Controller Started Successfully"

	# data logger
    if [ "$logging_enabled" = true ]; then
        start_component \
            "MorsLogger" \
            "${SCRIPT_DIR}/MorsLogger/build/mors_logger"
        logger_pgid="$LAST_PGID"

        setsid -- env --default-signal=INT,QUIT,TERM bash -c '
            sleep 120
            if kill -INT -- "-$1" 2>/dev/null; then
                echo "[MorsLogger]: stopped after 120 seconds"
            fi
        ' bash "$logger_pgid" &
        helper_groups+=("$!")
    fi

	wait
