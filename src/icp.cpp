#include "lidar_odometry/icp.hpp"
#include <limits>
#include <cmath>
#include <numeric>

namespace lidar_odometry
{

ICP::ICP(const ICPConfig & config)
: config_(config)
{}

void ICP::setConfig(const ICPConfig & config)
{
    config_ = config;
}

const ICPConfig & ICP::getConfig() const
{
    return config_;
}

// align function

ICPResult ICP::align(
    const std::vector<Point3D> & source,
    const std::vector<Point3D> & target,
    const Eigen::Matrix4f & initial_guess)
{
    ICPResult result;
    result.transform  = initial_guess;
    result.converged  = false;
    result.iterations = 0;
    result.final_error = std::numeric_limits<float>::max();

    // Need enough points to do anything meaningful
    if (source.size() < 10 || target.size() < 10)
    {
        result.transform = initial_guess;
        return result;
    }

    // Convert Point3D vectors to Eigen vectors once
    std::vector<Eigen::Vector3f> target_pts;
    target_pts.reserve(target.size());
    for (const auto & p : target)
        target_pts.push_back(p.toEigen());

    // Build the kd-tree on the target cloud once per align() call — the target
    // is fixed across ICP iterations, only the source moves.
    target_tree_.build(target_pts);

    // Start with source points transformed by initial guess
    std::vector<Eigen::Vector3f> source_pts;
    source_pts.reserve(source.size());
    for (const auto & p : source)
        source_pts.push_back(p.toEigen());

    source_pts = applyTransform(source_pts, initial_guess);

    // Accumulated transform starts as the initial guess
    Eigen::Matrix4f accumulated = initial_guess;

    for (int iter = 0; iter < config_.max_iterations; ++iter)
    {
        result.iterations = iter + 1;

        // find closest point pairs
        auto correspondences = findCorrespondences(source_pts);

        if (correspondences.empty())
            break;

        // compute best R, t for these pairs
        Eigen::Matrix4f delta = computeTransform(source_pts, target_pts, correspondences);

        // apply delta to source points
        source_pts = applyTransform(source_pts, delta);

        // Accumulate the full transform
        accumulated = delta * accumulated;

        // Compute error to check convergence
        float error = computeError(source_pts, target_pts, correspondences);
        result.final_error = error;

        // Check convergence: how big was this iteration's correction?
        // Extract translation magnitude and rotation angle from delta
        float translation_change = delta.block<3,1>(0,3).norm();
        float rotation_change    = std::acos(
            std::min(1.0f, (delta.block<3,3>(0,0).trace() - 1.0f) / 2.0f));

        if (translation_change < config_.convergence_tolerance &&
            rotation_change    < config_.convergence_tolerance)
        {
            result.converged = true;
            break;
        }
    }

    result.transform = accumulated;
    return result;
}


// Find nearest neighbour correspondences (KD-tree over target cloud).

std::vector<std::pair<int,int>> ICP::findCorrespondences(
    const std::vector<Eigen::Vector3f> & source_pts) const
{
    std::vector<std::pair<int,int>> correspondences;
    correspondences.reserve(source_pts.size());

    const float max_dist_sq = config_.max_correspondence_dist *
                              config_.max_correspondence_dist;

    for (int i = 0; i < static_cast<int>(source_pts.size()); ++i)
    {
        float best_dist_sq = 0.0f;
        const int best_j = target_tree_.nearest(source_pts[i], best_dist_sq);

        if (best_j >= 0 && best_dist_sq < max_dist_sq)
            correspondences.emplace_back(i, best_j);
    }

    return correspondences;
}


// Compute optimal R and t via SVD

Eigen::Matrix4f ICP::computeTransform(
    const std::vector<Eigen::Vector3f> & source_pts,
    const std::vector<Eigen::Vector3f> & target_pts,
    const std::vector<std::pair<int,int>> & correspondences) const
{
    const int N = static_cast<int>(correspondences.size());

    // Compute centroids
    Eigen::Vector3f src_centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f tgt_centroid = Eigen::Vector3f::Zero();

    for (const auto & [si, ti] : correspondences)
    {
        src_centroid += source_pts[si];
        tgt_centroid += target_pts[ti];
    }
    src_centroid /= N;
    tgt_centroid /= N;

    // Build cross-covariance matrix H
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();

    for (const auto & [si, ti] : correspondences)
    {
        Eigen::Vector3f ps = source_pts[si] - src_centroid;
        Eigen::Vector3f pt = target_pts[ti] - tgt_centroid;
        H += ps * pt.transpose();
    }

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // Handle reflection case (det = -1 means improper rotation)
    if ((V * U.transpose()).determinant() < 0.0f)
        V.col(2) *= -1.0f;

    // Optimal rotation
    Eigen::Matrix3f R = V * U.transpose();

    // Optimal translation
    Eigen::Vector3f t = tgt_centroid - R * src_centroid;

    // Pack into 4x4 homogeneous transform matrix
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;

    return T;
}

//  Apply transform to point set

std::vector<Eigen::Vector3f> ICP::applyTransform(
    const std::vector<Eigen::Vector3f> & points,
    const Eigen::Matrix4f & T) const
{
    std::vector<Eigen::Vector3f> result;
    result.reserve(points.size());

    const Eigen::Matrix3f R = T.block<3,3>(0,0);
    const Eigen::Vector3f t = T.block<3,1>(0,3);

    for (const auto & p : points)
        result.push_back(R * p + t);

    return result;
}


//  Compute mean squared error

float ICP::computeError(
    const std::vector<Eigen::Vector3f> & source_pts,
    const std::vector<Eigen::Vector3f> & target_pts,
    const std::vector<std::pair<int,int>> & correspondences) const
{
    if (correspondences.empty())
        return std::numeric_limits<float>::max();

    float total = 0.0f;
    for (const auto & [si, ti] : correspondences)
        total += (source_pts[si] - target_pts[ti]).squaredNorm();

    return total / static_cast<float>(correspondences.size());
}

} // namespace lidar_odometry