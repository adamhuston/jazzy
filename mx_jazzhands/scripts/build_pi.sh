#!/usr/bin/env bash
# Build the ROV2 workspace for the Raspberry Pi 3 B+ (armv7 / 32-bit) target.
#
# Portability is INTENTIONAL, not automatic: this is a separate build path from
# the x86_64 dev container. Two supported approaches:
#
#   1) Native build ON the Pi (simplest; slow):
#        - Install ROS 2 Jazzy (or a compatible build) on Raspberry Pi OS.
#        - Copy this repo to the Pi and run this script there.
#
#   2) Cross build via QEMU + docker buildx on the dev host:
#        docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
#        docker buildx build --platform linux/arm/v7 \
#          -f docker/Dockerfile.dev -t rov2/jazzy-armhf --load ..
#        # then run this script inside that arm/v7 container.
#
# Usage: scripts/build_pi.sh [extra colcon args...]
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd "$WS"

colcon build --symlink-install \
  --packages-select rov2_interfaces rov2_core rov2_plugins rov2_bringup \
  "$@"

echo
echo "arm32 build complete. Package it with scripts/package_pi.sh"
