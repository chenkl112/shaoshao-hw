#!/usr/bin/env bash
set -e
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
source /opt/ros/jazzy/setup.bash
source "$runtime_root/mapping_install/setup.bash"
if [[ $# -eq 0 || ( $# -eq 1 && $1 == --idle ) ]]; then
 exec python3 "$runtime_root/start_lidar_preview.py"
fi
exec python3 "$runtime_root/start_lidar_mapping.py" "$@"
