# LiDAR Odometry

A ROS 2 package that estimates a robot's 6-DoF pose in real time from 3D LiDAR scans using point-to-point **Iterative Closest Point (ICP)**. Implemented from scratch with only Eigen — no PCL, no Open3D.

---

## How it works

```
PointCloud2  ──►  Preprocessor  ──►  ICP (vs. previous scan)  ──►  Pose integration  ──►  /odom + TF
```

1. **Preprocess** — range filter, intensity filter, random downsampling to a fixed budget.
2. **ICP** — align the current scan to the previous one using nearest-neighbour correspondences, SVD-based rigid transform estimation (Kabsch), and a distance gate to reject outliers. The previous frame's delta is used as a warm-start initial guess.
3. **Integrate** — compose the per-scan delta onto the global pose, publish `nav_msgs/Odometry` on `/odom` and broadcast the `odom → base_link` TF.

## Topics & frames

| Direction | Topic   | Type                          |
|-----------|---------|-------------------------------|
| Subscribe | `/points` | `sensor_msgs/msg/PointCloud2` |
| Publish   | `/odom`   | `nav_msgs/msg/Odometry`       |
| TF        | `odom → base_link` | broadcast              |

## Parameters

Defined in [config/params.yaml](config/params.yaml):

| Group | Parameter | Default | Description |
|-------|-----------|---------|-------------|
| Preprocessor | `min_range` / `max_range` | 0.5 / 50.0 | Range gating (m) |
| | `intensity_threshold` | 0.0 | Reject weak returns (0 disables) |
| | `max_points` | 5000 | Downsample cap per scan |
| ICP | `max_iterations` | 50 | Iteration budget per scan pair |
| | `max_correspondence_dist` | 1.0 | Outlier rejection gate (m) |
| | `convergence_tolerance` | 1e-4 | Early-exit threshold on transform delta |
| Frames | `odom_frame` / `robot_frame` | `odom` / `base_link` | TF frame names |

## Build

Requires ROS 2 Humble, Eigen3, a C++17 compiler.

Clone into a ROS 2 workspace and build with colcon:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:adam979/LidarOdometry.git lidar_odometry
cd ~/ros2_ws
colcon build --packages-select lidar_odometry
source install/setup.bash
```

## Run

```bash
# Odometry only
ros2 launch lidar_odometry odometry.launch.py

# With RViz2 visualization
ros2 launch lidar_odometry odometry.launch.py rviz:=true
```

If your LiDAR publishes on a different topic, remap:

```bash
ros2 launch lidar_odometry odometry.launch.py \
  --ros-args -r points:=/your/lidar/topic
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
│   └── odometry.launch.py
├── include/lidar_odometry/
│   ├── preprocessor.hpp   # Point3D, PreprocessorConfig, Preprocessor
│   └── icp.hpp            # ICPConfig, ICPResult, ICP
└── src/
    ├── preprocessor.cpp
    ├── icp.cpp
    └── odometry_node.cpp  # ROS 2 node
```

The core (`preprocessor` + `icp`) is built as a ROS-independent library (`lidar_odometry_core`) and can be reused outside ROS.

## Limitations

- **Scan-to-scan only** — no local map or global optimization, so error accumulates over long trajectories.
- **Brute-force nearest neighbour** — O(N·M) per iteration. Keep `max_points` modest (≤5000) for real-time performance.
- **Point-to-point metric** — point-to-plane would converge faster and handle planar structures better.

## License

MIT
