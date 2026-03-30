#!/bin/bash

set -euo pipefail

(
    pids=()

    cleanup() {
        trap - SIGINT SIGTERM EXIT
        if [ ${#pids[@]} -gt 0 ]; then
            kill "${pids[@]}" 2>/dev/null || true
        fi
        pkill -P $$ 2>/dev/null || true
        wait 2>/dev/null || true
    }

    trap cleanup SIGINT SIGTERM EXIT

	echo "Launching Convex MPC Quadruped Controller..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "Script Location: $SCRIPT_DIR"

	config_dir="$SCRIPT_DIR/config/"
	echo "Config Location: ${config_dir}"

    # Fast DDS shared-memory transport often leaves stale locks in /dev/shm.
    # Force UDP transport for this local bringup to avoid SHM port conflicts.
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

    # ros2 controller
    ros2 launch robot_mode_controller bringup.launch.py &
    pids+=($!)

    # Проверяем аргумент
    if [ "${1:-}" == "--sim" ]; then
        echo "-------------- Simulation Mode Activated --------------" 
        ${SCRIPT_DIR}/.mpc_venv/bin/python ${SCRIPT_DIR}/Simulator/mors_simulator.py &
        pids+=($!)
        sleep 4s
    else
        # hardware interfaces
        echo "-------------- Hardware Mode Activated --------------"
        ${SCRIPT_DIR}/RealsenseCamera/build/realsense_camera  &
        pids+=($!)
        ${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu &
        pids+=($!)
        # in the future add here contact sensors controller
        # wait before the other controllers start to ensure that the hardware interfaces are up and running
        sleep 4s
    fi

    
    # state estimation
    # ${SCRIPT_DIR}/StateEstimator/build/state_estimator &
    # ${SCRIPT_DIR}/StateEstimatorLKF/build/state_estimator_lkf &

	# robot control
	${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC &
    pids+=($!)

    echo "Robot Controller Started Successfully"

	# data logger
    if [ "${2:-}" == "--log" ]; then
	    ${SCRIPT_DIR}/MorsLogger/build/mors_logger &
        logger_pid=$!
        pids+=($logger_pid)
        (
            sleep 60s
            kill "$logger_pid" 2>/dev/null || true
            echo "[MorsLogger]: killed"
        ) &
    fi
	
	wait
)
