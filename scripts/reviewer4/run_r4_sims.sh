#!/usr/bin/env bash
# Hakem 4 (yörünge kısmı) için koşular ve figürler.
#   bash $(rospack find multi_quadcopter_control)/scripts/reviewer4/run_r4_sims.sh
# 1) Planlayıcı karşılaştırması (PD+FF, downwash açık): log/r4_hybrid, log/r4_cubic_spline,
#    log/r4_minimum_jerk, log/r4_minimum_snap. Hibrit dışındakilerde tüm segmentler aynı yöntemle planlanır;
#    yeniden planlama segmentleri hibritteki gibi kalır (switch_segment), yani yol ve süre aynıdır.
# 2) Zaman ölçeği taraması (PD+FF, downwash açık, hibrit): log/r4_scale_<μ>, segment süreleri × μ.
# 3) Hesaplama süresi: TrajectorySequencer simülatörden bağımsız, tek iş parçacığında 200 kez koşulur
#    (log/r4_traj_eval/<yöntem>/timing.csv); donanım bilgisi log/r4_traj_eval/hardware.txt.
#    Aynı program referansın ivme, jerk ve snap değerlerini ve yeniden planlama anındaki sıçramaları yazar.
# 4) Figürler: scripts/Figures/reviewer4/FigR1_feasibility.svg, FigR2_segment_comparison.svg,
#    FigR3_computation_time.svg ve sayısal değerler (CSV).

PKG=$(rospack find multi_quadcopter_control) || { echo "Package not found. Did you source devel/setup.bash?"; exit 1; }
LOG="${PKG}/log"; EV="${LOG}/r4_traj_eval"; HERE="${PKG}/scripts/reviewer4"
COMMON="rviz:=false real_time_factor:=0 shutdown_on_finish:=true controller:=pid_ff"
VARIANTS="hybrid cubic_spline minimum_jerk minimum_snap"
SCALES="1.0 0.9 0.8 0.7 0.6 0.55 0.5 0.45 0.4"      # r4_figures.py ile aynı liste
mkdir -p "${EV}"
python3 "${HERE}/make_specs.py" "${PKG}/config" "${EV}"     # traj_eval girdileri ve all_trajectories_<yöntem>.yaml

for v in ${VARIANTS}; do
    yaml="${PKG}/config/all_trajectories.yaml"
    [ "${v}" != "hybrid" ] && yaml="${EV}/all_trajectories_${v}.yaml"
    echo; echo "=== planner ${v} -> ${LOG}/r4_${v}"
    roslaunch multi_quadcopter_control sequential_sim.launch ${COMMON} trajectories_file:="${yaml}" log_dir:="${LOG}/r4_${v}"
done

for k in ${SCALES}; do
    echo; echo "=== time scale ${k} -> ${LOG}/r4_scale_${k}"
    roslaunch multi_quadcopter_control sequential_sim.launch ${COMMON} trajectory_time_scale:="${k}" \
        log_dir:="${LOG}/r4_scale_${k}"
done

echo; echo "=== trajectory timing (${EV})"
CXXFLAGS="-std=c++17 -O2 -march=native"
g++ ${CXXFLAGS} -I"${PKG}/include" -I/usr/include/eigen3 "${HERE}/traj_eval.cpp" "${PKG}/src/sim/trajectory_sequencer.cpp" \
    -o "${EV}/traj_eval" || exit 1
for v in ${VARIANTS}; do
    mkdir -p "${EV}/${v}"
    "${EV}/traj_eval" "${EV}/spec_${v}.txt" "${EV}/${v}" 200
done
cpu=$(grep -m1 "model name" /proc/cpuinfo | sed -E 's/.*: //; s/\((R|TM)\)//g; s/ +/ /g; s/ CPU//; s/ Processor//')
{ echo "${cpu}, single thread"; echo "cores: $(nproc)"; g++ --version | head -1; echo "flags: ${CXXFLAGS}"; } > "${EV}/hardware.txt"
cat "${EV}/hardware.txt"

python3 "${HERE}/r4_figures.py" "${LOG}" "${EV}" "${PKG}/scripts/Figures/reviewer4"
