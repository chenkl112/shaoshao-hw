#!/usr/bin/env bash
set -e
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
source /opt/ros/jazzy/setup.bash
source "$runtime_root/mapping_install/setup.bash"
native="$runtime_root/mapping_install/lib/shaoshao_hw_runtime/hw_move_web"
[[ -x "$native" ]] || { echo "Native serial service is missing; run deployment/bootstrap.sh first" >&2; exit 1; }
exec "$native" --runtime "$runtime_root" --port 8081 --config "$runtime_root/config/serial.navcar.yaml" --hw-ros
