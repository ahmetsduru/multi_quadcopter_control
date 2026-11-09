#!/usr/bin/env bash
# Runs the simulations needed by scripts/main.m (manuscript figures and tables).
#   rosrun multi_quadcopter_control run_paper_sims.sh
# Each run writes droneN_log.txt files to its own folder inside the package.

PKG=$(rospack find multi_quadcopter_control) || { echo "Package not found. Did you source devel/setup.bash?"; exit 1; }
COMMON="rviz:=false real_time_factor:=0 shutdown_on_finish:=true"

run() {
    local label=$1 dir=$2; shift 2
    echo
    echo "=== ${label} -> ${dir}"
    roslaunch multi_quadcopter_control sequential_sim.launch ${COMMON} log_dir:="${dir}" "$@"
    if [ -f "${dir}/drone1_log.txt" ]; then
        echo "=== done: ${dir}"
    else
        echo "=== WARNING: no logs written to ${dir}"
    fi
}

run "[1/4] Proposed controller (PID + downwash feedforward)" "${PKG}/log"                controller:=pid_ff
run "[2/4] Ablation (PID, no feedforward; Fig. 8, Table 2)"  "${PKG}/log_no_feedforward" controller:=pid
run "[3/4] Yaw filter beta = 0.1 (Fig. S6)"                  "${PKG}/log_beta_0.1"       controller:=pid_ff psi_filter_beta:=0.1
run "[4/4] Geometric SE(3) controller (Fig. 2, Table 1)"    "${PKG}/log_se3"            controller:=se3

echo
echo "All runs finished. Now run scripts/main.m in MATLAB."
