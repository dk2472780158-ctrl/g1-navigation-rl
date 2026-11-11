# G1 FAST-LIO bridge

This package rotates the physically inverted G1 Mid360 point cloud and IMU by
180 degrees around X before FAST-LIO consumes them. The physical mounting TF
is kept separately, so the virtual FAST-LIO body frame remains upright.

Run the full mapping launch from the `slam` workspace:

```bash
ros2 launch g1_fastlio_bridge g1_fastlio_mapping.launch.py rviz:=true \
  map_file_path:=src/map/g1_fastlio.pcd
```

The default raw point cloud input is `/utlidar/cloud_livox_mid360` (Unitree
format) and the default raw IMU input is `/utlidar/imu_livox_mid360`. Both must
be `sensor_msgs/msg/PointCloud2` / `sensor_msgs/msg/Imu` topics for the G1 roll
correction nodes.

If the robot runs the official Livox driver instead, it publishes `/livox/lidar`
and `/livox/imu` with different point fields and per-point time. Override the
inputs at launch time and load the Livox driver FAST-LIO config
(`g1_mid360_livox_driver.yaml`, not shipped in this package) instead of
`g1_mid360_fastlio.yaml`, which is tuned for the Unitree topics:

```bash
ros2 launch g1_fastlio_bridge g1_fastlio_mapping.launch.py rviz:=true \
  input_cloud_topic:=/livox/lidar input_imu_topic:=/livox/imu \
  fast_lio_config_file:=g1_mid360_livox_driver.yaml
```

The IMU input is republished to `/g1_fastlio/imu_livox_mid360_upright` with a
valid frame ID, SI acceleration, and the same roll correction as the point
cloud.

The G1 stream observed on the robot contains XYZI fields but no per-point
`ring`, `line`, or `time`. The included FAST-LIO configuration uses its generic
PointCloud2 handler and estimates the point offsets from their order at 10 Hz.
Keep `scan_rate` equal to the actual `/utlidar/cloud_livox_mid360` publication rate.

The save directory is created on demand. The map path can be relative to the
current workspace or absolute.
