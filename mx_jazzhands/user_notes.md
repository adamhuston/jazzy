## WSL (PowerShell → Ubuntu 24.04 / ROS 2 Jazzy)

We run the framework natively in WSL (Ubuntu 24.04, ROS 2 Jazzy). No Docker.

Enter WSL from PowerShell:

`wsl`                          # default distro
`wsl -d Ubuntu-24.04`          # a specific distro

Enter and load ROS 2 (underlay only):

`wsl bash -lc "source /opt/ros/jazzy/setup.bash && exec bash"`

Enter and load ROS 2 + our overlay:

`wsl bash -lc "source /opt/ros/jazzy/setup.bash && source /mnt/c/projects/jazzy/mx_jazzhands/install/setup.bash 2>/dev/null; exec bash"`

Typical sim session (inside WSL):

```
cd /mnt/c/projects/jazzy/mx_jazzhands
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/c/projects/isaacsim/config/fastdds_loopback_peer.xml
ros2 launch rov2_bringup rov2.launch.py profile:=sim
```

The `sim` profile now sets `use_sim_time: true` on its own, so `profile:=sim` is
enough. Override explicitly with `use_sim_time:=true` only when using a different
profile against a simulator.

Activate alive loop:
`ros2 run rov2_core rov2_core_node`

NOTE (paths): `/mnt/c/projects/jazzy/mx_jazzhands` is the Windows repo seen from
WSL — convenient for editing on Windows, but colcon builds are noticeably faster
from a native clone under the WSL filesystem (e.g. `~/jazzy/mx_jazzhands`). Pick
one and stay consistent so the overlay you source matches what you built.


## Sourcing (inside WSL)

Two layers must be sourced. Every NEW shell needs the underlay; any shell that
uses our custom messages/plugins also needs the overlay.

- Underlay (base ROS 2 — gives you `ros2`, `colcon`):
  `source /opt/ros/jazzy/setup.bash`
- Overlay (our built workspace — gives you `rov2_*` types/plugins):
  `source /mnt/c/projects/jazzy/mx_jazzhands/install/setup.bash`

GOTCHA: `The message/service type 'rov2_interfaces/...' is invalid` in a
`ros2 topic echo` / `service call` shell means the OVERLAY isn't sourced in
THAT shell. The running node is fine; the CLI is a separate process.

Optional convenience — auto-source both in every WSL shell (append to `~/.bashrc`):
```
source /opt/ros/jazzy/setup.bash
[ -f /mnt/c/projects/jazzy/mx_jazzhands/install/setup.bash ] && \
  source /mnt/c/projects/jazzy/mx_jazzhands/install/setup.bash
```


## Build (inside WSL, from mx_jazzhands/)

- Build everything:
  `colcon build`
- Build only what changed / specific packages:
  `colcon build --packages-select rov2_interfaces rov2_core rov2_plugins`
- Re-source the overlay after every successful build:
  `source install/setup.bash`
- Clean rebuild (when CMake/interfaces act stale):
  `rm -rf build install log && colcon build`

Helper scripts (from the repo root, inside WSL):
- Build: `scripts/build_dev.sh`
- Launch a profile: `scripts/run_dev.sh sim`   (or `dev` / `hardware`)

If a fresh clone drops the executable bit, either run `chmod +x scripts/*.sh`
once, or invoke with `bash scripts/build_dev.sh`.


## Run

- Core only (no plugins):
  `ros2 run rov2_core rov2_core_node`
- Core with the mock plugins loaded:
  ```
  ros2 run rov2_core rov2_core_node --ros-args \
    -p sensor_plugins:="[rov2_plugins::MockSensor]" \
    -p actuator_plugins:="[rov2_plugins::MockActuator]" \
    -p brain_plugins:="[rov2_plugins::NoopBrain]"
  ```


## Inspect / validate (needs underlay + overlay sourced)

- `ros2 topic echo /rov2_core/status`      framework state, heartbeat, loop jitter, plugins[]
- `ros2 topic echo /diagnostics`           per-plugin + core health
- `ros2 lifecycle get /rov2_core`          current managed state (expect `active [3]`)
- `ros2 node list` / `ros2 topic list`
- `ros2 interface show rov2_interfaces/msg/SystemStatus`
- Set mode (0=STANDBY 1=ACTIVE 2=SAFE):
  `ros2 service call /rov2_core/set_mode rov2_interfaces/srv/SetMode "{mode: 2, reason: 'test'}"`
- Send a motion command (forwarded to actuators when mode=ACTIVE):
  `ros2 topic pub -1 /rov2_core/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"`


## Git (PowerShell, from C:\projects\jazzy)

- Clone:  `git clone https://github.com/adamhuston/jazzy.git`
- Pull latest (run before building):  `git pull`
- Typical commit + push:
  ```
  git add -A
  git commit -m "message"
  git push origin main
  ```


## WSL management (PowerShell)

- List distros:            `wsl -l -v`
- Enter default distro:    `wsl`
- Enter a specific distro: `wsl -d Ubuntu-24.04`
- Shut WSL down:           `wsl --shutdown`


## Reference

- Runtime: WSL 2, Ubuntu 24.04, ROS 2 Jazzy (native — no Docker).
- Repo path in WSL: `/mnt/c/projects/jazzy/mx_jazzhands` (or a native `~/jazzy/mx_jazzhands` clone).
- ROS distro: Jazzy | RMW: rmw_fastrtps_cpp (Fast DDS)
- For remote Isaac Sim interop: matching `ROS_DOMAIN_ID` + same RMW on both machines.
- Legacy: a Docker dev setup still lives under `docker/` but is no longer the
  supported workflow.