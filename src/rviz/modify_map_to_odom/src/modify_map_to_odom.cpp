#include "modify_map_to_odom/modify_map_to_odom.hpp"

#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace modify_map_to_odom
{
ModifyMapToOdom::ModifyMapToOdom(const rclcpp::NodeOptions &options)
    : Node("modify_map_to_odom", options)
{
    parent_frame_ = declare_parameter<std::string>("parent_frame", "map");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    alignment_.x = declare_parameter<double>("X", 0.0);
    alignment_.y = declare_parameter<double>("Y", 0.0);
    alignment_.theta = declare_parameter<double>("Rotation", 0.0);
    const auto publish_rate = declare_parameter<double>("publish_rate", 10.0);
    auto finite = [](const geometry_msgs::msg::Pose2D &value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.theta);
    };
    if (!finite(alignment_) || !std::isfinite(publish_rate) || publish_rate <= 0.0 ||
        parent_frame_.empty() || target_frame_.empty() ||
        parent_frame_ == target_frame_)
        throw std::invalid_argument("Invalid map to odom configuration");

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // The publisher owns the current alignment, including after an RViz reconnect.
    alignment_publisher_ = create_publisher<geometry_msgs::msg::Pose2D>(
        "/map_odom/current", rclcpp::QoS(1).transient_local());
    alignment_publisher_->publish(alignment_);
    alignment_subscription_ = create_subscription<geometry_msgs::msg::Pose2D>(
        "/map_odom/set", 10,
        [this, finite](const geometry_msgs::msg::Pose2D::SharedPtr value) {
            if (!finite(*value)) return;
            alignment_ = *value;
            alignment_publisher_->publish(alignment_);
            publish_transform();
        });
    publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                                     [this]() { publish_transform(); });
}

void ModifyMapToOdom::publish_transform()
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = target_frame_;
    transform.transform.translation.x = alignment_.x;
    transform.transform.translation.y = alignment_.y;
    transform.transform.rotation.z = std::sin(alignment_.theta / 2);
    transform.transform.rotation.w = std::cos(alignment_.theta / 2);
    broadcaster_->sendTransform(transform);
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(modify_map_to_odom::ModifyMapToOdom)
