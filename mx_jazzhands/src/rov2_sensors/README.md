# Reusable hardware sensors

`rov2_sensors::Rplidar` and `rov2_sensors::Adxl345` implement
`rov2_core::SensorPlugin`. They report observations and acquisition health,
not collision zones, impact decisions, tilt decisions, or motion authority.
Robot wiring and policy belong in the consuming robot's bringup package.

## Acquisition and lifecycle

Configuration validates parameters without opening hardware. Activation
starts one acquisition worker and a 10 ms publication timer. The core's
`poll()` never performs device I/O. ADXL345 FIFO batches are drained into
individual messages, including transient peaks; publication is not limited
to the core's 20 Hz loop. Each connection owns its device until deactivation.
Deactivation joins the worker, stops the LiDAR scan/motor or places the
accelerometer in standby, and invalidates observations.

`ACTIVE` means fresh acquisition, **not** safety readiness or calibration.
`DEGRADED` means missing/stale samples; `FAULT` means a transport, device,
decoding, saturation, or overflow error. Device errors are logged and retried
after one second. An acquisition stall is also retried after
`max(1000 ms, 4 * max_staleness_ms)`. Reconnection discards old samples.
Hardware-free activation never invents observations.

Workers cannot block the core, but teardown must wait for an in-flight
operation. LiDAR scan reads wait at most 100 ms; identification/health/stop
requests use 200 ms timeouts. SDK startup includes additional bounded SDK
requests (the SDK's default timeout is 2 seconds). Linux I2C operations are
bounded by the adapter/kernel timeout, not a userspace hard deadline; verify
the target's bus-fault/shutdown latency on hardware.

## Topics and health

Data uses best-effort, volatile, keep-last(5). Health uses reliable, volatile,
keep-last(10) `diagnostic_msgs/DiagnosticArray`, on both `/diagnostics` and
the configurable per-sensor `health_topic`. Health updates every 100 ms,
including when data stops. Diagnostic names are `rov2_sensors/rplidar` and
`rov2_sensors/adxl345`.

Fields: `connected`, `identified`, `streaming`, `fresh`, `sample_age_ms`,
`max_staleness_ms`, `sequence`, `reconnect_count`, `dropped_samples`,
`last_error`, and `timestamp_basis`. LiDAR additionally reports `valid_bins`,
`unknown_bins`, and `valid_fraction`. Missing samples have age `-1`.
Consumers must watchdog health messages too, and validate observation
timestamps/geometry independently. Fresh data does not imply full coverage,
known mounting, reliable calibration, or permission to move.

Old messages are never restamped or republished. Monotonic acquisition time
governs freshness; ROS stamps are mapped at acquisition to support ordinary
ROS consumers. Outdated buffered data is dropped, not stamped as new.

### RPLIDAR

Parameters under `rplidar`:

| Parameter | Default |
|---|---|
| `device`, `baud` | `/dev/ttyUSB0`, `115200` |
| `topic`, `frame_id` | `/scan`, `laser` |
| `health_topic`, `max_staleness_ms` | `/sensors/rplidar/health`, `250.0` |
| `range_min_m`, `range_max_m` | `0.15`, `12.0` |

Standard scans use the upstream Slamtec SDK; a successful device-info reply
and healthy status are required before starting the motor. The Linux SDK's
first-ray timestamp is in `CLOCK_MONOTONIC`. Its actual age, including SDK
queuing, is mapped to ROS/steady time. The first revolution establishes scan
duration and is not published.

Angles follow Slamtec's ROS convention, `pi - SDK_angle`. A **negative**
angle increment preserves clockwise acquisition order and positive temporal
increments instead of reversing time. Measurements are conservatively
rebinned into evenly spaced angular cells; duplicate bins retain the nearest
return. Per-ray times are estimates for those cells. Unknown, zero-quality,
zero-distance, out-of-range, and unobserved cells are **NaN**, not clear space.
A scan with no usable returns is rejected. Partial scans remain partial:
consumers must assess valid coverage in their own zones.

The A1's 250 ms freshness budget includes a complete scan revolution
(nominally about 182 ms at 5.5 Hz). Validate actual age and jitter on the rover.

### ADXL345

Parameters under `adxl345`:

| Parameter | Default |
|---|---|
| `device`, `address` | `/dev/i2c-1`, `83` (`0x53`; also supports `0x1d`) |
| `topic`, `frame_id` | `/accel`, `accel_link` |
| `health_topic`, `max_staleness_ms` | `/sensors/adxl345/health`, `100.0` |
| `range_g`, `rate_hz` | `16`, `100` (supports 2/4/8/16 g and 50/100/200 Hz) |
| `bias_mps2`, `scale` | `[0,0,0]`, `[1,1,1]` |
| `acceleration_variance` | `0.01` in `(m/s^2)^2` |

Identity must be `0xe5`. Full-resolution right-justified samples use the
datasheet's 3.9 mg/LSB scale, converted to m/s^2. Calibration is
`(raw_mps2 - bias_mps2) * scale` in sensor axes. Gravity is retained.
Stream FIFO is polled every 5 ms; oldest-first timestamps are conservatively
estimated from FIFO depth and configured output rate. FIFO full/overrun and
saturated observations are rejected explicitly, never silently flattened.

The `sensor_msgs/Imu` message marks orientation and angular velocity unavailable
with covariance element 0 equal to `-1`. This is **not** a yaw/gyro/full-pose
sensor. Tilt estimation is valid only under appropriate gravity/static-motion
assumptions and belongs to a processing consumer.

## SDK and build

The SDK is fetched at commit
`99478e5fb90de3b4a6db0080acacd373f8b36869`, built as a static Linux library,
and linked into the plugin. No separate ROS LiDAR driver should open that
serial device concurrently. First configure needs network access to public
upstream source; offline builders can pass
`-DFETCHCONTENT_SOURCE_DIR_SLAMTEC=/path/to/the/pinned/sdk/checkout`.
Its BSD-2-Clause notice is installed as `share/rov2_sensors/SLAMTEC-LICENSE`.
The SDK includes Linux support; x86 WSL is tested here, ARM compilation and
physical operation remain target verification steps.

From the framework workspace in WSL Ubuntu-24.04:

```bash
bash scripts/build_dev.sh --packages-up-to rov2_sensors
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select rov2_sensors --event-handlers console_direct+
colcon test-result --test-result-base build/rov2_sensors --verbose
```

Tests use fake register I/O and fake acquisition sources, verify ROS data/health
publication, and test scan conversion. They do not certify physical scan timing,
USB behavior, mounting, acceleration accuracy, or impact coverage.
