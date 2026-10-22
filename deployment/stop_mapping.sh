#!/usr/bin/env bash
set -e
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
source /opt/ros/jazzy/setup.bash
source "$runtime_root/mapping_install/setup.bash"
native="$runtime_root/mapping_install/lib/shaoshao_hw_runtime/hw_runtime"
[[ -x "$native" ]] || { echo "Native runtime is missing; run deployment/bootstrap.sh first" >&2; exit 1; }
exec "$native" stop-mapping --runtime "$runtime_root"
