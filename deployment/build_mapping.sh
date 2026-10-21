#!/usr/bin/env bash
set -e
script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_root/../shaoshao_hw/config.py" ]]; then
  exec bash "$script_root/bootstrap.sh"
fi
exec bash "$script_root/shaoshao/deployment/bootstrap.sh"
