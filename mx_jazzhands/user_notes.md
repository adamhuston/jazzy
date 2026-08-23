## Powershell

Enter the jazzy environment:

(prereq: the environment is running in docker and you don't want to use the docker terminal)

`docker exec -it jazzy-dev bash`


Enter and loads ROS2:

`docker exec -it jazzy-dev bash -c "source /opt/ros/jazzy/setup.bash && exec bash"`


Activate alive loop:
`ros2 run rov2_core rov2_core_node`


## Sourcing (inside the container)

Two layers must be sourced. Every NEW shell needs the underlay; any shell that
uses our custom messages/plugins also needs the overlay.

- Underlay (base ROS 2 — gives you `ros2`, `colcon`):
  `source /opt/ros/jazzy/setup.bash`
- Overlay (our built workspace — gives you `rov2_*` types/plugins):
  `source /workspace/jazzy/mx_jazzhands/install/setup.bash`

GOTCHA: `The message/service type 'rov2_interfaces/...' is invalid` in a
`ros2 topic echo` / `service call` shell means the OVERLAY isn't sourced in
THAT shell. The running node is fine; the CLI is a separate process.

Optional convenience — auto-source both on entry:
`docker exec -it jazzy-dev bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/jazzy/mx_jazzhands/install/setup.bash 2>/dev/null; exec bash"`


## Build (inside the container, from mx_jazzhands/)

- Build everything:
  `colcon build`
- Build only what changed / specific packages:
  `colcon build --packages-select rov2_interfaces rov2_core rov2_plugins`
- Re-source the overlay after every successful build:
  `source install/setup.bash`
- Clean rebuild (when CMake/interfaces act stale):
  `rm -rf build install log && colcon build`


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
- Pull latest (run inside the container before building):  `git pull`
- Typical commit + push:
  ```
  git add -A
  git commit -m "message"
  git push origin main
  ```


## Docker (PowerShell)

- List running containers:  `docker ps`
- Enter the container:       `docker exec -it jazzy-dev bash`
- Start it if stopped:       `docker start jazzy-dev`


## Reference

- Container name: `jazzy-dev`  (image `ros:jazzy`)
- Repo path in container: `/workspace/jazzy/mx_jazzhands`
- ROS distro: Jazzy | RMW: rmw_fastrtps_cpp (Fast DDS)
- For remote Isaac Sim interop: matching `ROS_DOMAIN_ID` + same RMW on both machines.