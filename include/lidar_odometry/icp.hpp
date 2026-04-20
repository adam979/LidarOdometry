#pragma once

#include <vector>
#include <Eigen/Dense>
#include "lidar_odometry/preprocessor.hpp"

namespace lidar_odometry
{

struct ICPConfig
{
    int   max_iterations;          // maximum ICP iterations per scan pair
    float max_correspondence_dist; // metres - ignore point pairs farther than this
    float convergence_tolerance;   // stop if transform change is below this
};

struct ICPResult
{
    Eigen::Matrix4f transform;     // 4x4 rigid transform that aligns source to target
    float           final_error;   // mean squared error after alignment
    int             iterations;    // how many iterations it took
    bool            converged;     // did it converge within max_iterations?
};

class ICP
{
public:
    explicit ICP(const ICPConfig & config);

    // Align source cloud to target cloud
    // initial_guess: warm start transform (use previous result for better convergence)
    // Returns the transform that maps source → target
    ICPResult align(
        const std::vector<Point3D> & source,
        const std::vector<Point3D> & target,
        const Eigen::Matrix4f & initial_guess = Eigen::Matrix4f::Identity());

    void setConfig(const ICPConfig & config);
    const ICPConfig & getConfig() const;

private:
    ICPConfig config_;

    // for each source point find nearest target point
    // Returns pairs of (source_idx, target_idx)
    std::vector<std::pair<int,int>> findCorrespondences(
        const std::vector<Eigen::Vector3f> & source_pts,
        const std::vector<Eigen::Vector3f> & target_pts) const;

    // compute optimal R and t from correspondences using SVD
    Eigen::Matrix4f computeTransform(
        const std::vector<Eigen::Vector3f> & source_pts,
        const std::vector<Eigen::Vector3f> & target_pts,
        const std::vector<std::pair<int,int>> & correspondences) const;

    // apply a 4x4 transform to a set of points
    std::vector<Eigen::Vector3f> applyTransform(
        const std::vector<Eigen::Vector3f> & points,
        const Eigen::Matrix4f & T) const;

    // Compute mean squared error over correspondences
    float computeError(
        const std::vector<Eigen::Vector3f> & source_pts,
        const std::vector<Eigen::Vector3f> & target_pts,
        const std::vector<std::pair<int,int>> & correspondences) const;
};

} // namespace lidar_odometry