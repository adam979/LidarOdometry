#include "lidar_odometry/kdtree.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace lidar_odometry
{

void KDTree::build(const std::vector<Eigen::Vector3f> & points)
{
    points_ = points;
    root_.reset();

    if (points_.empty())
        return;

    std::vector<int> indices(points_.size());
    std::iota(indices.begin(), indices.end(), 0);

    root_ = buildRec(indices, 0);
}

std::unique_ptr<KDTree::Node> KDTree::buildRec(
    std::vector<int> & indices, int depth)
{
    if (indices.empty())
        return nullptr;

    const int axis = depth % 3;
    const size_t mid = indices.size() / 2;

    // Partition indices so that the median on `axis` lands at position `mid`.
    // nth_element is O(n) expected, so total build is O(n log n).
    std::nth_element(
        indices.begin(),
        indices.begin() + mid,
        indices.end(),
        [this, axis](int a, int b)
        {
            return points_[a][axis] < points_[b][axis];
        });

    auto node  = std::make_unique<Node>();
    node->idx  = indices[mid];
    node->axis = axis;

    std::vector<int> left_idx (indices.begin(),             indices.begin() + mid);
    std::vector<int> right_idx(indices.begin() + mid + 1,   indices.end());

    node->left  = buildRec(left_idx,  depth + 1);
    node->right = buildRec(right_idx, depth + 1);

    return node;
}

int KDTree::nearest(const Eigen::Vector3f & query, float & best_sq_dist) const
{
    int   best_idx = -1;
    float best_sq  = std::numeric_limits<float>::max();

    nearestRec(root_.get(), query, best_idx, best_sq);

    best_sq_dist = best_sq;
    return best_idx;
}

void KDTree::nearestRec(
    const Node * n,
    const Eigen::Vector3f & q,
    int & best_idx,
    float & best_sq) const
{
    if (n == nullptr)
        return;

    const Eigen::Vector3f & p = points_[n->idx];
    const float d_sq = (q - p).squaredNorm();

    if (d_sq < best_sq)
    {
        best_sq  = d_sq;
        best_idx = n->idx;
    }

    const int axis = n->axis;
    const float delta = q[axis] - p[axis];

    // Descend the side of the hyperplane the query is on first — it usually
    // contains the true nearest, making pruning on the other side effective.
    const Node * near_side = (delta < 0.0f) ? n->left.get()  : n->right.get();
    const Node * far_side  = (delta < 0.0f) ? n->right.get() : n->left.get();

    nearestRec(near_side, q, best_idx, best_sq);

    // Only visit the far side if the splitting hyperplane could host a closer
    // point than our current best (standard KD-tree backtracking criterion).
    if (delta * delta < best_sq)
        nearestRec(far_side, q, best_idx, best_sq);
}

} // namespace lidar_odometry
