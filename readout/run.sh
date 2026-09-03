#!/usr/bin/env bash
# Launch jazzwatch on the Pi: source the ROS 2 overlay, activate the venv, run.
#
# Usage:
#   ./run.sh              # live ROS data + battery
#   ./run.sh --mock       # synthetic data (no ROS, e.g. over plain SSH)
#   ./run.sh --no-battery # skip serial reads
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# ROS 2 Jazzy overlay (system install + this workspace's install/).
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$HERE/../install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HERE/../install/setup.bash"
fi

# Project venv (created with --system-site-packages so rclpy is visible).
if [[ -f "$HERE/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "$HERE/.venv/bin/activate"
fi

exec python3 -m jazzy_readout "$@"
