#include "lidar_odometry/preprocessor.hpp"
#include <algorithm>

namespace lidar_odometry
{

Preprocessor::Preprocessor(const PreprocessorConfig & config)
: config_(config)
{}

void Preprocessor::setConfig(const PreprocessorConfig & config)
{
    config_ = config;
}

const PreprocessorConfig & Preprocessor::getConfig() const
{
    return config_;
}

std::vector<Point3D> Preprocessor::filter(const std::vector<Point3D> & raw_points) const
{
    std::vector<Point3D> filtered;
    filtered.reserve(raw_points.size());

    for (const auto & p : raw_points)
    {
        const float r = p.range();

        // Range filter
        if (r < config_.min_range || r > config_.max_range)
            continue;

        // Intensity filter
        if (p.intensity < config_.intensity_threshold)
            continue;

        filtered.push_back(p);
    }

    // Downsample if needed - take evenly spaced points
    if (config_.max_points > 0 &&
        static_cast<int>(filtered.size()) > config_.max_points)
    {
        std::vector<Point3D> downsampled;
        downsampled.reserve(config_.max_points);

        const float step = static_cast<float>(filtered.size()) /
                           static_cast<float>(config_.max_points);

        for (int i = 0; i < config_.max_points; ++i)
        {
            downsampled.push_back(filtered[static_cast<int>(i * step)]);
        }

        return downsampled;
    }

    return filtered;
}

} // namespace lidar_odometry