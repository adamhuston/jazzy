#!/usr/bin/env bash
# Build the ROV2 workspace in the Jazzy WSL/Ubuntu 24.04 environment (x86_64).
# Usage: scripts/build_dev.sh [extra colcon args...]
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS setup.bash references unbound vars; relax nounset only while sourcing it.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
cd "$WS"

colcon build --symlink-install "$@"

echo
echo "Build complete. Source the overlay before running:"
echo "  source \"$WS/install/setup.bash\""
