#ifndef MODIFY_MAP_TO_ODOM__MODIFY_MAP_TO_ODOM_HPP
#define MODIFY_MAP_TO_ODOM__MODIFY_MAP_TO_ODOM_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <QSharedMemory>
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

namespace modify_map_to_odom
{
    class ModifyMapToOdom : public rclcpp::Node
    {
    public:
        explicit ModifyMapToOdom(const rclcpp::NodeOptions &options);
        ~ModifyMapToOdom();

    private:
        double yaw;
        double pitch;
        double roll;
        double x;
        double y;

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr alignment_subscription;
        std::string parent_frame;
        std::string target_frame;

        Eigen::Isometry3d map_to_odom;

        rclcpp::TimerBase::SharedPtr map_to_odom_publish_timer;
        rclcpp::TimerBase::SharedPtr listen_keyboard_timer;
        std::mutex map_to_odom_lock;
        QSharedMemory XYR_memory_list;

        double *directionArray = nullptr;
        // double *direction;
        void map_to_odom_publish_callback();
        void listen_keyboard();
    };
} // namespace ModifyMapToOdom

#endif
