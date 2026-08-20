#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: nextage_topic_bridge.sh CONFIG_YAML [NODE_NAME]" >&2
  exit 2
fi

config_yaml="$1"
node_name="${2:-parameter_bridge}"

if [[ ! -r "${config_yaml}" ]]; then
  echo "nextage_topic_bridge.sh: cannot read ${config_yaml}" >&2
  exit 1
fi

rosparam load "${config_yaml}"
exec ros2 run ros1_bridge parameter_bridge "__name:=${node_name}"
