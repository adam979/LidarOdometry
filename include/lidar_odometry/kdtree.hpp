#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace lidar_odometry
{

// 3D KD-tree over Eigen::Vector3f points. Axis-aligned, cycles x→y→z with depth.
// Exact nearest-neighbour search via recursive descent + backtracking through the
// splitting hyperplane (visit far side only if it could contain a closer point).
class KDTree
{
public:
    KDTree() = default;

    // Copy points and build the tree. O(n log n) expected.
    void build(const std::vector<Eigen::Vector3f> & points);

    // Returns index of nearest stored point to query, and sets best_sq_dist.
    // Returns -1 if tree is empty.
    int nearest(const Eigen::Vector3f & query, float & best_sq_dist) const;

    size_t size() const { return points_.size(); }
    bool   empty() const { return points_.empty(); }

private:
    struct Node
    {
        int idx;   // index into points_
        int axis;  // 0=x, 1=y, 2=z
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    std::vector<Eigen::Vector3f> points_;
    std::unique_ptr<Node>        root_;

    std::unique_ptr<Node> buildRec(std::vector<int> & indices, int depth);

    void nearestRec(
        const Node * n,
        const Eigen::Vector3f & q,
        int & best_idx,
        float & best_sq) const;
};

} // namespace lidar_odometry
