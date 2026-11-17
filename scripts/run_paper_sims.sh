#!/usr/bin/env bash
# Runs the simulations needed by scripts/main.m (manuscript and supplement figures and tables).
#   rosrun multi_quadcopter_control run_paper_sims.sh
# Every run writes droneN_log.txt and run_info.txt to its own folder log/<run>/:
#   log/pid_ff, log/pid, log/ndob, log/se3, log/se3_ff   one folder per controller (disturbed)
#   log/pid_nominal, log/se3_nominal                     PID / SE(3), disturbance off
#   log/pid_ff_beta0.1                                   PID+FF, yaw filter beta = 0.1
# main.m picks the runs it needs by these names (settings at the top of main.m).

PKG=$(rospack find multi_quadcopter_control) || { echo "Package not found. Did you source devel/setup.bash?"; exit 1; }
LOG="${PKG}/log"
COMMON="rviz:=false real_time_factor:=0 shutdown_on_finish:=true"

run() {
    local label=$1 name=$2; shift 2
    local dir="${LOG}/${name}"
    echo
    echo "=== ${label} -> ${dir}"
    roslaunch multi_quadcopter_control sequential_sim.launch ${COMMON} log_dir:="${dir}" "$@"
    if [ -f "${dir}/drone1_log.txt" ]; then
        echo "=== done: ${dir}"
    else
        echo "=== WARNING: no logs written to ${dir}"
    fi
}

run "[1/8] PID + downwash feedforward (PID+FF)"     pid_ff         controller:=pid_ff
run "[2/8] PID, no feedforward"                     pid            controller:=pid
run "[3/8] PID + NDOB, no downwash model"           ndob           controller:=ndob
run "[4/8] Geometric SE(3), no feedforward"         se3            controller:=se3
run "[5/8] Geometric SE(3) + feedforward"           se3_ff         controller:=se3_ff
run "[6/8] PID, disturbance off"                    pid_nominal    controller:=pid disturbance_enabled:=false
run "[7/8] SE(3), disturbance off"                  se3_nominal    controller:=se3 disturbance_enabled:=false
run "[8/8] PID+FF, yaw filter beta = 0.1"           pid_ff_beta0.1 controller:=pid_ff psi_filter_beta:=0.1

echo
echo "All runs finished. Now run scripts/main.m in MATLAB."
