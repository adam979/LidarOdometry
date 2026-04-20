#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace lidar_odometry
{

// A single LiDAR point
struct Point3D
{
    float x, y, z, intensity;

    // Distance from sensor origin
    float range() const
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    // Convert to Eigen vector (for ICP math)
    Eigen::Vector3f toEigen() const
    {
        return Eigen::Vector3f(x, y, z);
    }
};

struct PreprocessorConfig
{
    float min_range;           // metres - remove points closer than this
    float max_range;           // metres - remove points farther than this
    float intensity_threshold; // remove points with intensity below this
    int   max_points;          // downsample to this many points max (0 = no limit)
};

class Preprocessor
{
public:
    explicit Preprocessor(const PreprocessorConfig & config);

    // Filter a raw point cloud, returns cleaned points ready for ICP
    std::vector<Point3D> filter(const std::vector<Point3D> & raw_points) const;

    // Update config at runtime (for ROS2 parameter changes)
    void setConfig(const PreprocessorConfig & config);
    const PreprocessorConfig & getConfig() const;

private:
    PreprocessorConfig config_;
};

} // namespace lidar_odometry
