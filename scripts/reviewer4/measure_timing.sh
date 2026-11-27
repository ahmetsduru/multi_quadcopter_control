#!/usr/bin/env bash
# FigR3'ü (yörünge üretecinin hesaplama süresi) bu bilgisayarda ölçer ve çizer. ROS gerekmez:
# g++, Eigen3 ve python3 (numpy, pandas, matplotlib) yeterli.
#   bash scripts/reviewer4/measure_timing.sh [tekrar_sayısı]      (varsayılan 200)
# Çıktılar: log/r4_traj_eval/ (ölçümler, hardware.txt) ve scripts/Figures/reviewer4/FigR3_computation_time.svg
# Ölçüm sırasında başka ağır programları kapatın; dizüstü bilgisayarda şarj kablosu takılı olsun.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PKG=$(cd "${HERE}/../.." && pwd)
REPS=${1:-200}
EV=${R4_EV:-"${PKG}/log/r4_traj_eval"}
OUT=${R4_OUT:-"${PKG}/scripts/Figures/reviewer4"}
mkdir -p "${EV}" "${OUT}"

# Çizim için gereken Python paketleri ölçümden önce denetlenir
if ! python3 -c "import numpy, pandas, matplotlib" 2>/dev/null; then
    echo "Eksik Python paketi (numpy, pandas, matplotlib). Ubuntu'da:"
    echo "  sudo apt install python3-numpy python3-pandas python3-matplotlib fonts-liberation"
    exit 1
fi

CXXFLAGS="-std=c++17 -O2 -march=native"
EIGEN=$(pkg-config --cflags eigen3 2>/dev/null || echo "-I/usr/include/eigen3")
echo "=== derleme"
g++ ${CXXFLAGS} -I"${PKG}/include" ${EIGEN} "${HERE}/traj_eval.cpp" "${PKG}/src/sim/trajectory_sequencer.cpp" -o "${EV}/traj_eval"
python3 "${HERE}/make_specs.py" "${PKG}/config" "${EV}"
for v in hybrid cubic_spline minimum_jerk minimum_snap; do
    echo "=== ${v}: ${REPS} tekrar"
    mkdir -p "${EV}/${v}"
    "${EV}/traj_eval" "${EV}/spec_${v}.txt" "${EV}/${v}" "${REPS}"
done

cpu=$(grep -m1 "model name" /proc/cpuinfo 2>/dev/null | sed -E 's/.*: //')
[ -z "${cpu}" ] && cpu=$(sysctl -n machdep.cpu.brand_string 2>/dev/null || echo "unknown CPU")
cpu=$(echo "${cpu}" | sed -E 's/\((R|TM)\)//g; s/ CPU//; s/ Processor//; s/ +/ /g; s/^ //; s/ $//')
{
    echo "${cpu}, single thread"                          # ilk satır figüre yazılır
    echo "cores: $(nproc 2>/dev/null || sysctl -n hw.ncpu)"
    echo "memory: $(free -h 2>/dev/null | awk '/Mem:/{print $2}')"
    echo "os: $( (. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME}") || uname -sr)"
    echo "compiler: $(g++ --version | head -1)"
    echo "flags: ${CXXFLAGS}"
    echo "repetitions: ${REPS}"
} > "${EV}/hardware.txt"
echo "=== donanım"; cat "${EV}/hardware.txt"

R4_ONLY=R3 python3 "${HERE}/r4_figures.py" "${PKG}/log" "${EV}" "${OUT}"
echo "=== figür: ${OUT}/FigR3_computation_time.svg"
echo "=== sayılar: ${OUT}/FigR3_timing_Q${R4_DRONE:-2}.csv"
