#!/usr/bin/env bash
set -e
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
source /opt/ros/jazzy/setup.bash
source "$runtime_root/mapping_install/setup.bash"
native="$runtime_root/mapping_install/lib/shaoshao_hw_runtime/hw_runtime"
[[ -x "$native" ]] || { echo "Native runtime is missing; run deployment/bootstrap.sh first" >&2; exit 1; }
if [[ $# -eq 0 || ( $# -eq 1 && $1 == --idle ) ]]; then
 exec "$native" preview --runtime "$runtime_root"
fi
for argument in "$@"; do
  case "$argument" in
    --keep-web|--expected-pid|--expected-maps-dir|--filter-config) ;;
    *) echo "Unsupported native mapping option: $argument" >&2; exit 2;;
  esac
done
exec "$native" start-mapping --runtime "$runtime_root"
