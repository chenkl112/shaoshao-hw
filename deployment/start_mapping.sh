#!/usr/bin/env bash
set -e
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
source /opt/ros/jazzy/setup.bash
source "$runtime_root/mapping_install/setup.bash"
native="$runtime_root/mapping_install/shaoshao_hw_runtime/lib/shaoshao_hw_runtime/hw_runtime"
[[ -x "$native" ]] || native="$runtime_root/mapping_install/lib/shaoshao_hw_runtime/hw_runtime"
[[ -x "$native" ]] || { echo "Native runtime is missing; run deployment/bootstrap.sh first" >&2; exit 1; }
if [[ $# -eq 0 || ( $# -eq 1 && $1 == --idle ) ]]; then
 exec "$native" preview --runtime "$runtime_root"
fi
if [[ $# -eq 1 && $1 == --start ]]; then
 "$native" preview --runtime "$runtime_root"
 exec "$native" start-mapping --runtime "$runtime_root"
fi
if [[ $# -eq 1 && $1 == --check ]]; then
 exec "$native" status --runtime "$runtime_root"
fi
echo "Usage: start_mapping.sh [--idle|--start|--check]" >&2
exit 2
