#!/usr/bin/env bash
# Build the ROV2 workspace inside the Jazzy dev container (x86_64).
# Usage: scripts/build_dev.sh [extra colcon args...]
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd "$WS"

colcon build --symlink-install "$@"

echo
echo "Build complete. Source the overlay before running:"
echo "  source \"$WS/install/setup.bash\""
