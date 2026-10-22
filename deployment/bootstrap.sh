#!/usr/bin/env bash
set -euo pipefail
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_root="${SHAOSHAO_HW_RUNTIME:-$HOME/ccc/shaoshao-hw-runtime}"
mkdir -p "$runtime_root" "$source_root/third_party"
fetch_dependency() {
  local name="$1" repo="$2" revision="$3" destination="$source_root/third_party/$1"
  if [[ -f "$destination/.shaoshao-commit" ]]; then
    [[ "$(tr -d '\r\n' < "$destination/.shaoshao-commit")" == "$revision" ]] || { echo "Dependency revision mismatch: $name" >&2; exit 1; }
  else
    [[ ! -e "$destination" ]] || { echo "Unverified dependency directory: $destination" >&2; exit 1; }
    git init "$destination"
    git -C "$destination" fetch --depth 1 "https://github.com/$repo.git" "$revision"
    git -C "$destination" checkout --detach FETCH_HEAD
    [[ "$(git -C "$destination" rev-parse HEAD)" == "$revision" ]]
    echo "$revision" > "$destination/.shaoshao-commit"
  fi
}
fetch_dependency small_gicp koide3/small_gicp 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e
fetch_dependency Livox-SDK2 Livox-SDK/Livox-SDK2 c0796f04c143143899c87a773d9f6b7136453c0b
cmake -S "$source_root/third_party/Livox-SDK2" -B "$runtime_root/sdk_build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build "$runtime_root/sdk_build" -j2
if [[ "$(id -u)" == 0 ]]; then
  cmake --install "$runtime_root/sdk_build"
  ldconfig
else
  sudo cmake --install "$runtime_root/sdk_build"
  sudo ldconfig
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$source_root"
colcon --log-base "$runtime_root/build_logs" build \
  --base-paths FAST_LIO livox_ros_driver2 autonomous_exploration_development_environment/src/terrain_analysis_ext \
    shaoshao_hw_interfaces hw_registration hw_omni_controller shaoshao_hw_runtime shaoshao_web \
  --build-base "$runtime_root/build" --install-base "$runtime_root/mapping_install" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
if [[ -L "$runtime_root/shaoshao" ]]; then
  [[ "$(readlink -f "$runtime_root/shaoshao")" == "$source_root" ]] || { echo 'Runtime belongs to another checkout' >&2; exit 1; }
elif [[ -e "$runtime_root/shaoshao" ]]; then
  echo 'Runtime source alias already exists; no files overwritten' >&2; exit 1
else ln -s "$source_root" "$runtime_root/shaoshao"; fi
for entry in "$source_root/deployment/"*; do
  case "$(basename "$entry")" in
    config|mapping_runtime)
      mkdir -p "$runtime_root/$(basename "$entry")"
      if [[ "$(basename "$entry")" == mapping_runtime ]]; then
        cp -a "$entry/mapping.launch.py" "$runtime_root/mapping_runtime/"
      fi
      ;;
    *) cp -a "$entry" "$runtime_root/" ;;
  esac
done
python3 "$source_root/deployment/setup_runtime.py" "$source_root" "$runtime_root"
chmod +x "$runtime_root/"*.sh
echo "Built Shaoshao HW. Runtime: $runtime_root"
