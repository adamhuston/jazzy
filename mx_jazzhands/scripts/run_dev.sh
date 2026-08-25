#!/usr/bin/env bash
# Launch the ROV2 core runtime with a bringup profile.
# Usage: scripts/run_dev.sh [profile]     (profile: dev | sim | hardware, default dev)
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS setup.bash references unbound vars; relax nounset only while sourcing.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Overlay not found. Run scripts/build_dev.sh first." >&2
  exit 1
fi
source "$WS/install/setup.bash"
set -u

PROFILE="${1:-dev}"
exec ros2 launch rov2_bringup rov2.launch.py profile:="$PROFILE"
