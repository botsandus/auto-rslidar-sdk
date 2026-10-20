# Safety Airy prototype with ROS 2

Use `RSSAFETYAIRY` for the pre-production Safety Airy protocol. `RSAIRY` uses a
different packet layout and cannot decode this unit.

Build the SDK with its updated `rs_driver` submodule and the `rslidar_msg`
dependency available in the ROS environment:

```sh
colcon build --packages-select rslidar_sdk \
  --cmake-args -DPOINT_TYPE=XYZIRTF -DCOMPILE_TESTS=ON
source install/setup.bash
ros2 launch rslidar_sdk safety_airy.launch.py
```

The installed `config/safety_airy.yaml` is a single-sensor bench example. It
binds to host address `192.168.1.102` and receives UDP ports 6699, 7788 and 6688.
Set the host address to the address of the computer's lidar-facing interface.
Pass another configuration with `config_path:=/absolute/path/config.yaml`.
The driver receives data; it does not reconfigure the sensor or safety fields.

## Published topics

- `/safety_airy/points`: `sensor_msgs/msg/PointCloud2`, approximately 10 Hz.
  Fields are x/y/z/intensity (float32), ring (uint16, 0–95), t (uint32,
  nanoseconds from the cloud header stamp), and feature (uint8, the two raw range
  flag bits). `XYZIRTF` uses a 28-byte aligned point layout. Consumers should
  use the message's field offsets. Keep `ts_first_point: true` for unsigned t.
- `/safety_airy/imu`: `sensor_msgs/msg/Imu`, approximately 200 Hz. Acceleration
  is m/s² and angular velocity is rad/s. `orientation_covariance[0] = -1`
  indicates that no attitude estimate is available.
- `/safety_airy/diagnostics`: `diagnostic_msgs/msg/DiagnosticArray`, 10 Hz.
  Includes raw OSSD states, field indicators, active group, faults, firmware,
  clock status, calibration/extrinsic validity, packet counts and sequence gaps.
  Missing or older-than-0.5-second valid DIFOP is reported as STALE. Raw OSSD
  OFF states produce WARN; the diagnostic level is not a safety permit.
- `/safety_airy/packets`: `rslidar_msg/msg/RslidarPacket`, raw MSOP and DIFOP,
  when `send_packet_ros` is enabled. Original payload timestamps and CRCs are
  preserved, including when cloud/IMU publishing uses the host clock.

The cloud frame is `safety_airy`. IMU data uses the raw `safety_airy_imu` frame.
The captured IMU extrinsic quaternion is all zero. No transform or orientation
is invented from that value; provide a separately verified mounting transform.

With `dense_points: false`, invalid/zero/out-of-range samples remain NaNs.
`feature` is opaque: no validity or quality meaning is assigned to its bits.
The 0.1–60 m limits are decoder limits, not guaranteed safety detection ranges.

## Clock and supported profile

The tested profile is 96 channels, data class 0, single-return byte 1, and
type/model 0x10/0x02. Type 0x01 is also accepted as the manual's identifier.
Other model/return profiles are rejected. MSOP is 1240 bytes, DIFOP 1248 bytes
and IMU 51 bytes. MSOP/DIFOP CRC validation is mandatory for this model.

`use_lidar_clock: false` anchors the common sensor clock to host epoch time and
retains sensor-relative intervals, avoiding per-packet receipt jitter in point
offsets. It reanchors after a clock discrepancy over two seconds and discards
the partial cloud. This is not precision synchronization. Enable lidar-clock
timestamps only after validating synchronization on the actual system.

Optical offsets, factory timing and range masking follow the supplied
SafeParamTool 3.6.0 decoder. Calibration comes from the device's DIFOP. Physical
geometry and deskew timing still need validation against targets/vendor output.
Per-column times use the nominal factory schedule. Encoder jumps at sector
boundaries are not converted into extra time: recorded packet timestamps do
not consistently support that inference. Do not treat this timing as a
validated deskew model at those boundaries.
Alternative firing schedules, dual returns, detailed range-flag meanings and
ambiguous fault-bit mappings are outside the validated profile.

For two sensors, configure different destination port sets for MSOP, DIFOP and
IMU on the sensors and in their YAML entries. `host_address` selects a local
receive interface; it is not a source-IP filter. This driver does not write the
sensor network settings.

## Checks

`ctest --test-dir build/rslidar_sdk --output-on-failure` runs the registered
Safety decoder and ROS conversion/diagnostic tests when built with
`COMPILE_TESTS=ON` and `POINT_TYPE=XYZIRTF`.

The core fixture tests use real packets from the prototype, known CRC values,
golden coordinates and malformed packets. They do not certify the sensor or
turn ROS publishing into a functional-safety communication channel.
