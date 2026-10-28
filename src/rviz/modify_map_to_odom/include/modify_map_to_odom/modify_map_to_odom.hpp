#ifndef MODIFY_MAP_TO_ODOM__MODIFY_MAP_TO_ODOM_HPP
#define MODIFY_MAP_TO_ODOM__MODIFY_MAP_TO_ODOM_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace modify_map_to_odom
{
class ModifyMapToOdom : public rclcpp::Node
{
public:
    explicit ModifyMapToOdom(const rclcpp::NodeOptions &options);

private:
    geometry_msgs::msg::Pose2D alignment_;
    std::string parent_frame_, target_frame_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr alignment_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr alignment_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    void publish_transform();
};
}
#endif
