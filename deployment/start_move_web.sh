#!/usr/bin/env bash
set -e
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
source /opt/ros/jazzy/setup.bash
source "$runtime_root/mapping_install/setup.bash"
cd "$runtime_root/shaoshao"
exec python3 -u web_panel/navcar_move_web.py --port 8081 --hw-ros --config "$runtime_root/config/serial.navcar.yaml"
