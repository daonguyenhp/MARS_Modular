#!/usr/bin/env bash
# Open the Mode 1 RViz playback. Default map is geogebra.
#   ./04_demo_simulation/run.sh
#   ./04_demo_simulation/run.sh --map hard_alley
#   ./04_demo_simulation/run.sh --svg
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map="geogebra"
svg=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      map="${2:?missing map id after --map}"
      shift 2
      ;;
    --svg)
      svg=1
      shift
      ;;
    *)
      echo "Usage: $0 [--map hard_alley|geogebra] [--svg]" >&2
      exit 2
      ;;
  esac
done

if [[ "${svg}" -eq 1 ]]; then
  build="${root}/.build/mars"
  cmake -S "${root}" -B "${build}" -DCMAKE_BUILD_TYPE=Debug
  cmake --build "${build}" --target polygon_explore_sim -j2
  bin="${build}/04_demo_simulation/polygon_explore_sim"
  if [[ ! -x "${bin}" && -x "${build}/04_demo_simulation/Debug/polygon_explore_sim.exe" ]]; then
    bin="${build}/04_demo_simulation/Debug/polygon_explore_sim.exe"
  elif [[ ! -x "${bin}" && -x "${build}/04_demo_simulation/Release/polygon_explore_sim.exe" ]]; then
    bin="${build}/04_demo_simulation/Release/polygon_explore_sim.exe"
  fi
  cd "${root}"
  exec "${bin}" --map "${map}"
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS 1 Noetic not found at /opt/ros/noetic" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

export ROS_HOME="${root}/.build/ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
export PYTHONUNBUFFERED=1
mkdir -p "${ROS_LOG_DIR}"

ws="${root}/.build/mars_ws"
mkdir -p "${ws}/src"
ln -sfn "${root}/04_demo_simulation/ros1/mars_mode1_sim" "${ws}/src/mars_mode1_sim"
cd "${ws}"
catkin_make -j2
# shellcheck disable=SC1091
source "${ws}/devel/setup.bash"
exec roslaunch mars_mode1_sim polygon_explore_sim.launch "map:=${map}"
