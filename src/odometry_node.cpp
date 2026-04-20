#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "lidar_odometry/preprocessor.hpp"
#include "lidar_odometry/icp.hpp"

namespace lidar_odometry
{

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode()
    : Node("lidar_odometry_node")
    {
        // Declare all parameters 
        // Preprocessor
        this->declare_parameter("min_range",            0.5);
        this->declare_parameter("max_range",           50.0);
        this->declare_parameter("intensity_threshold",  0.0);
        this->declare_parameter("max_points",         5000);

        // ICP
        this->declare_parameter("max_iterations",          50);
        this->declare_parameter("max_correspondence_dist",  1.0);
        this->declare_parameter("convergence_tolerance",    1e-4);

        // Frame names
        this->declare_parameter("odom_frame",  std::string("odom"));
        this->declare_parameter("robot_frame", std::string("base_link"));

        // Build preprocessor and ICP from parameters
        preprocessor_ = std::make_unique<Preprocessor>(getPreprocessorConfig());
        icp_          = std::make_unique<ICP>(getICPConfig());

        // Global pose starts at identity
        global_pose_ = Eigen::Matrix4f::Identity();
        last_transform_ = Eigen::Matrix4f::Identity();
        first_scan_received_ = false;

        // TF2 broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Odometry publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // PointCloud2 subscriber
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points",
            rclcpp::SensorDataQoS(),
            std::bind(&OdometryNode::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LiDAR Odometry node started.");
        RCLCPP_INFO(this->get_logger(), "Listening on topic: points");
    }

private:
    // Members 
    std::unique_ptr<Preprocessor>              preprocessor_;
    std::unique_ptr<ICP>                       icp_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    std::vector<Point3D> previous_cloud_;
    Eigen::Matrix4f      global_pose_;      // accumulated world transform
    Eigen::Matrix4f      last_transform_;   // last ICP delta (warm start)
    bool                 first_scan_received_;

    // Convert PointCloud2 message → vector<Point3D>
    std::vector<Point3D> fromROSMsg(
        const sensor_msgs::msg::PointCloud2 & msg) const
    {
        std::vector<Point3D> points;
        points.reserve(msg.width * msg.height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        // Intensity field may not always be present
        bool has_intensity = false;
        for (const auto & field : msg.fields)
            if (field.name == "intensity") { has_intensity = true; break; }

        if (has_intensity)
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_i(msg, "intensity");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i)
            {
                if (!std::isfinite(*iter_x) ||
                    !std::isfinite(*iter_y) ||
                    !std::isfinite(*iter_z)) continue;

                Point3D p;
                p.x = *iter_x; p.y = *iter_y;
                p.z = *iter_z; p.intensity = *iter_i;
                points.push_back(p);
            }
        }
        else
        {
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                if (!std::isfinite(*iter_x) ||
                    !std::isfinite(*iter_y) ||
                    !std::isfinite(*iter_z)) continue;

                Point3D p;
                p.x = *iter_x; p.y = *iter_y;
                p.z = *iter_z; p.intensity = 1.0f;
                points.push_back(p);
            }
        }

        return points;
    }

    // Main callback 
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Parse and filter the incoming cloud
        const std::string cloud_frame = msg->header.frame_id;
        auto raw    = fromROSMsg(*msg);
        auto filtered = preprocessor_->filter(raw);

        RCLCPP_DEBUG(this->get_logger(),
            "Raw: %zu pts → Filtered: %zu pts",
            raw.size(), filtered.size());

        if (filtered.size() < 10)
        {
            RCLCPP_WARN(this->get_logger(), "Too few points after filtering, skipping.");
            return;
        }

        // First scan — store and publish identity
        if (!first_scan_received_)
        {
            previous_cloud_      = filtered;
            first_scan_received_ = true;
            publishPose(msg->header.stamp, cloud_frame);
            return;
        }

        // Run ICP: align current scan to previous scan
        //    Use last transform as warm start for better convergence
        ICPResult result = icp_->align(filtered, previous_cloud_, last_transform_);

        RCLCPP_DEBUG(this->get_logger(),
            "ICP: %d iters, error=%.4f, converged=%s",
            result.iterations, result.final_error,
            result.converged ? "yes" : "no");

        // Accumulate pose: global = global * delta
        global_pose_   = global_pose_ * result.transform;
        last_transform_ = result.transform;

        // Update previous cloud
        previous_cloud_ = filtered;

        // Publish TF2 + odometry
        publishPose(msg->header.stamp, cloud_frame);
    }

    // Publish TF2 transform and odometry message
    void publishPose(
        const rclcpp::Time & stamp,
        const std::string & child_frame)
    {
        const std::string odom_frame =
            this->get_parameter("odom_frame").as_string();

        // Extract R and t from global pose
        const Eigen::Matrix3f R = global_pose_.block<3,3>(0,0);
        const Eigen::Vector3f t = global_pose_.block<3,1>(0,3);

        // Convert rotation matrix to quaternion
        Eigen::Quaternionf q(R);
        q.normalize();

        // TF2 broadcast 
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = stamp;
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id  = child_frame;

        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();
        tf_msg.transform.rotation.x    = q.x();
        tf_msg.transform.rotation.y    = q.y();
        tf_msg.transform.rotation.z    = q.z();
        tf_msg.transform.rotation.w    = q.w();

        tf_broadcaster_->sendTransform(tf_msg);

        // Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = stamp;
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id  = child_frame;

        odom_msg.pose.pose.position.x = t.x();
        odom_msg.pose.pose.position.y = t.y();
        odom_msg.pose.pose.position.z = t.z();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom_msg);
    }

    // Build configs from ROS2 parameters
    PreprocessorConfig getPreprocessorConfig() const
    {
        PreprocessorConfig cfg;
        cfg.min_range           = this->get_parameter("min_range").as_double();
        cfg.max_range           = this->get_parameter("max_range").as_double();
        cfg.intensity_threshold = this->get_parameter("intensity_threshold").as_double();
        cfg.max_points          = this->get_parameter("max_points").as_int();
        return cfg;
    }

    ICPConfig getICPConfig() const
    {
        ICPConfig cfg;
        cfg.max_iterations         = this->get_parameter("max_iterations").as_int();
        cfg.max_correspondence_dist = this->get_parameter("max_correspondence_dist").as_double();
        cfg.convergence_tolerance  = this->get_parameter("convergence_tolerance").as_double();
        return cfg;
    }
};

} // namespace lidar_odometry

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_odometry::OdometryNode>());
    rclcpp::shutdown();
    return 0;
}