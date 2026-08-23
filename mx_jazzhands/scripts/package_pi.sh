#!/usr/bin/env bash
# Package the built workspace into a deployable artifact for the Raspberry Pi.
# Produces a tarball of install/ that can be extracted and sourced on the Pi.
# Usage: scripts/package_pi.sh [output.tar.gz]
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

if [ ! -d install ]; then
  echo "No install/ directory. Run scripts/build_pi.sh first." >&2
  exit 1
fi

OUT="${1:-rov2_pi_$(date +%Y%m%d_%H%M%S).tar.gz}"
tar -czf "$OUT" install

echo "Wrote $OUT"
echo
echo "Deploy on the Pi:"
echo "  tar -xzf $(basename "$OUT")"
echo "  source install/setup.bash"
echo "  ros2 launch rov2_bringup rov2.launch.py profile:=hardware"
