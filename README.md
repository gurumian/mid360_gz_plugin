# mid360_gz_plugin

Gazebo Sim (gz-sim) system plugin that simulates **Livox Mid360** LiDAR using a CSV-defined non-repetitive scan pattern. It publishes `sensor_msgs/PointCloud2` on a ROS 2 topic.

This package is intended to be kept in its own git repository and used as a dependency where needed.

## Dependencies

- ROS 2 (Jazzy / Humble)
- Gazebo Sim (gz-sim 8, Harmonic/Citadel)
- `sensor_msgs`, `rclcpp`
- gz_sim_vendor, gz_plugin_vendor, gz_transport_vendor, gz_msgs_vendor

## Build

```bash
cd /path/to/workspace
colcon build --packages-select mid360_gz_plugin
source install/setup.bash
```

## Plugin path

Add the plugin library to Gazebo’s search path:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(ros2 pkg prefix mid360_gz_plugin)/lib/mid360_gz_plugin
```

## SDF usage

Attach the plugin to a **link** that represents the Mid360 sensor (e.g. `lidar_link`). The CSV path can be absolute or relative; for the installed config use the package share path.

Example (inside a `<model>`):

```xml
<link name="lidar_link">
  <pose>0 0 0.1 0 0 0</pose>
  <inertial>
    <mass>0.1</mass>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  <visual name="mid360_visual">
    <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
  </visual>
  <collision name="mid360_collision">
    <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
  </collision>
  <plugin filename="mid360_gz_plugin" name="mid360_gz_plugin::Mid360Plugin">
    <frame_id>lidar_link</frame_id>
    <ros_topic>points</ros_topic>
    <csv_file>PATH_TO_MID360_CSV</csv_file>
    <min_range>0.1</min_range>
    <max_range>200.0</max_range>
    <samples>24000</samples>
    <downsample>1</downsample>
  </plugin>
</link>

<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent>
  <child>lidar_link</child>
</joint>
```

**CSV path:** After install, the default scan pattern is at  
`$(ros2 pkg prefix mid360_gz_plugin)/share/mid360_gz_plugin/config/mid360.csv`.  
Use that path (or pass an absolute path) in `<csv_file>`.

### Parameters

| Parameter    | Type   | Default      | Description                          |
|-------------|--------|--------------|--------------------------------------|
| `frame_id`  | string | `livox_mid360` | Frame ID for the PointCloud2 header. |
| `ros_topic` | string | `points`    | ROS 2 topic for PointCloud2.        |
| `csv_file`  | string | (required)  | Path to Mid360 scan pattern CSV.     |
| `min_range` | double | 0.1         | Minimum range (m).                  |
| `max_range` | double | 200.0       | Maximum range (m).                  |
| `samples`   | int    | 24000       | Number of rays per update.          |
| `downsample`| int    | 1           | Downsample factor (≥1).             |

## Current behavior

- Loads the scan pattern from the CSV (columns: Time/s, Azimuth/deg, Zenith/deg).
- Each update, uses `samples`/`downsample` ray directions and cycles through the CSV.
- Uses gz-sim’s **RaycastData** component: in PreUpdate the plugin sets ray start/end in the link frame; the physics raycast system fills results; in PostUpdate the plugin reads hit points and publishes PointCloud2 in the link frame. If no hit (e.g. `fraction` ≥ 1), the point is taken at `max_range` along the ray.
- **CSV path:** You can pass an absolute path or a `package://` URI (e.g. `package://mid360_gz_plugin/config/mid360.csv`); the plugin resolves it via `ament_index_cpp`.

## License

Apache-2.0
