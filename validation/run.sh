#!/usr/bin/env bash
set -euo pipefail
source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SHAOSHAO_HW_RUNTIME="${SHAOSHAO_HW_RUNTIME:-/workspace/runtime}"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=72 SHAOSHAO_HW_INTEGRATION=1
bash "$source_root/deployment/bootstrap.sh"
set +u
source /opt/ros/jazzy/setup.bash
source "$SHAOSHAO_HW_RUNTIME/mapping_install/setup.bash"
set -u
cd "$source_root"
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s web_panel -p 'test*.py' -q
g++ -std=c++17 -I FAST_LIO/include FAST_LIO/test/test_static_map_filter.cpp -o "$SHAOSHAO_HW_RUNTIME/static_filter_test"
"$SHAOSHAO_HW_RUNTIME/static_filter_test"
g++ -std=c++17 -I FAST_LIO/include FAST_LIO/test/test_map_ceiling_filter.cpp -o "$SHAOSHAO_HW_RUNTIME/ceiling_filter_test"
"$SHAOSHAO_HW_RUNTIME/ceiling_filter_test"
python3 -u validation/ros_integration.py
SHAOSHAO_HW_RUNTIME="$SHAOSHAO_HW_RUNTIME/lio-validation" python3 -u validation/lio_mapping_integration.py
