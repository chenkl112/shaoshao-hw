#!/usr/bin/env bash
set -euo pipefail
source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SHAOSHAO_HW_RUNTIME="${SHAOSHAO_HW_RUNTIME:-/workspace/runtime}"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=72 SHAOSHAO_HW_INTEGRATION=1
export SHAOSHAO_BUILD_TESTING=ON
bash "$source_root/deployment/bootstrap.sh"
set +u
source /opt/ros/jazzy/setup.bash
source "$SHAOSHAO_HW_RUNTIME/mapping_install/setup.bash"
set -u
cd "$source_root"
ctest --test-dir "$SHAOSHAO_HW_RUNTIME/build/shaoshao_hw_runtime" --output-on-failure
ctest --test-dir "$SHAOSHAO_HW_RUNTIME/build/hw_omni_controller" --output-on-failure
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s web_panel -p 'test*.py' -q
g++ -O2 -std=c++20 -I /usr/include/eigen3 \
  -I hw_omni_controller/include -I hw_omni_controller/vendor \
  hw_omni_controller/test/test_mpc.cpp -o "$SHAOSHAO_HW_RUNTIME/mpc_test"
"$SHAOSHAO_HW_RUNTIME/mpc_test"
g++ -O2 -std=c++20 -I /usr/include/eigen3 -I hw_omni_controller/include \
  hw_omni_controller/test/test_runtime.cpp -o "$SHAOSHAO_HW_RUNTIME/runtime_test"
"$SHAOSHAO_HW_RUNTIME/runtime_test"
cmake -S shaoshao_web -B "$SHAOSHAO_HW_RUNTIME/web_native_check" \
  -DSHAOSHAO_WEB_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$SHAOSHAO_HW_RUNTIME/web_native_check" -j2
python3 validation/web_integration.py "$SHAOSHAO_HW_RUNTIME/web_native_check/shaoshao_web"
node web_panel/validation/navcar_move_ui.cjs
node web_panel/validation/mobile_mapping_ui.cjs
node web_panel/validation/lidar_mapping_ui.cjs
node web_panel/validation/stream_ui.cjs
g++ -std=c++17 -I FAST_LIO/include FAST_LIO/test/test_static_map_filter.cpp -o "$SHAOSHAO_HW_RUNTIME/static_filter_test"
"$SHAOSHAO_HW_RUNTIME/static_filter_test"
g++ -std=c++17 -I FAST_LIO/include FAST_LIO/test/test_map_ceiling_filter.cpp -o "$SHAOSHAO_HW_RUNTIME/ceiling_filter_test"
"$SHAOSHAO_HW_RUNTIME/ceiling_filter_test"
SHAOSHAO_HW_RUNTIME="$SHAOSHAO_HW_RUNTIME/navcar-v2" ROS_DOMAIN_ID=73 python3 -u validation/navcar_v2_integration.py
python3 -u validation/ros_integration.py
SHAOSHAO_HW_RUNTIME="$SHAOSHAO_HW_RUNTIME/lio-validation" python3 -u validation/lio_mapping_integration.py
SHAOSHAO_MAPPING_SUPERVISOR=1 SHAOSHAO_HW_RUNTIME="$SHAOSHAO_HW_RUNTIME/mapping-supervised" python3 -u validation/lio_mapping_integration.py
