# LiDAR Odometry

A ROS 2 package that estimates a robot's 6-DoF pose in real time from 3D LiDAR scans using point-to-point **Iterative Closest Point (ICP)**. Implemented from scratch with only Eigen — no PCL, no Open3D.

---

## How it works

```
PointCloud2  ──►  Preprocessor  ──►  ICP (vs. previous scan)  ──►  Pose integration  ──►  /odom + TF (+ estimate.txt)
```

1. **Preprocess** — range filter, intensity filter, and downsampling to a fixed point budget.
2. **ICP** — align the current scan to the previous one using nearest-neighbour correspondences (KD-tree over the target cloud), SVD-based rigid transform estimation (Kabsch), and a distance gate to reject outliers. The previous frame's delta is used as a warm-start initial guess.
3. **Integrate** — compose the per-scan delta onto the global pose, broadcast the `odom → cloud_frame` TF, publish `nav_msgs/Odometry` on `/odom`, and optionally append the pose to a TUM-format trajectory file.

## Topics & frames

| Direction | Topic   | Type                          |
|-----------|---------|-------------------------------|
| Subscribe | `points` (default `/points`, remappable) | `sensor_msgs/msg/PointCloud2` |
| Publish   | `/odom`   | `nav_msgs/msg/Odometry`       |
| TF        | `odom → <cloud_frame>` | broadcast (child = incoming cloud's `frame_id`) |

## Trajectory dump

When `dump_estimate` is `true` (the default), the node writes the estimated trajectory to `estimate_path` (default `./estimate.txt`) in space-separated TUM format, one pose per scan:

```
cloud_timestamp tx ty tz qx qy qz qw
```

## Parameters

Defined in [config/params.yaml](config/params.yaml):

| Group | Parameter | Default | Description |
|-------|-----------|---------|-------------|
| Preprocessor | `min_range` / `max_range` | 0.5 / 50.0 | Range gating (m) |
| | `intensity_threshold` | 0.0 | Reject weak returns (0 disables) |
| | `max_points` | 7500 | Downsample cap per scan (0 = no limit) |
| ICP | `max_iterations` | 40 | Iteration budget per scan pair |
| | `max_correspondence_dist` | 1.25 | Outlier rejection gate (m) |
| | `convergence_tolerance` | 1e-4 | Early-exit threshold on transform delta |
| Frames | `odom_frame` | `odom` | Parent TF frame (child is the cloud's `frame_id`) |
| Dump | `dump_estimate` | `true` | Write the TUM-format trajectory file |
| | `estimate_path` | `./estimate.txt` | Output path for the trajectory |

## Build

Requires ROS 2 Humble, Eigen3, and a C++17 compiler.

Clone into a ROS 2 workspace and build with a single colcon call:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:adam979/LidarOdometry.git st196428
cd ~/ros2_ws
colcon build --packages-select st196428
source install/setup.bash
```

## Run

```bash
# Odometry only
ros2 launch st196428 odom.launch.py

# With RViz2 visualization
ros2 launch st196428 odom.launch.py rviz:=true
```

If your LiDAR publishes on a different topic, point the node at it with the `input_topic` launch argument:

```bash
ros2 launch st196428 odom.launch.py input_topic:=/your/lidar/topic
```

## Project layout

```
.
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── params.yaml        # runtime parameters
│   └── odometry.rviz      # RViz2 configuration
├── launch/
│   └── odom.launch.py
├── include/lidar_odometry/
│   ├── preprocessor.hpp   # Point3D, PreprocessorConfig, Preprocessor
│   ├── icp.hpp            # ICPConfig, ICPResult, ICP
│   └── kdtree.hpp         # KDTree (nearest-neighbour search)
└── src/
    ├── preprocessor.cpp
    ├── icp.cpp
    ├── kdtree.cpp
    └── odometry_node.cpp  # ROS 2 node
```

The core (`preprocessor` + `icp` + `kdtree`) is built as a ROS-independent library (`lidar_odometry_core`) that depends only on Eigen and can be reused outside ROS.

## Limitations

- **Scan-to-scan only** — no local map or global optimization, so error accumulates over long trajectories.
- **Point-to-point metric** — point-to-plane would converge faster and handle planar structures better.
- If bag replay outpaces ICP, slow the replay down (`--rate`, or the arrow keys in `ros2 bag play`).

## License

MIT
