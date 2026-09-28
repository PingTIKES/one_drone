#include "modify_map_to_odom/modify_map_to_odom.hpp"

#include <tf2/impl/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

#include <termios.h>
#include <fcntl.h>

namespace modify_map_to_odom
{
    ModifyMapToOdom::ModifyMapToOdom(const rclcpp::NodeOptions &options)
        : Node("modify_map_to_odom", options)
    {
        double direction[3] = {0.0, 0.0, 0.0};
        this->declare_parameter<std::string>("parent_frame", "map");
        this->declare_parameter<std::string>("target_frame", "odom");
        this->declare_parameter<double>("X", 0.0);
        this->declare_parameter<double>("Y", 0.0);
        this->declare_parameter<double>("Rotation", 0.0);
        this->get_parameter<std::string>("parent_frame", parent_frame);
        this->get_parameter<std::string>("target_frame", target_frame);
        this->get_parameter<double>("X", direction[0]);
        this->get_parameter<double>("Y", direction[1]);
        this->get_parameter<double>("Rotation", direction[2]);

        std::cout << direction[0] << " " << direction[1] << " " << direction[2] << std::endl;
        // 创建共享内存
        XYR_memory_list.setKey("one_drone_direction");

        if (!XYR_memory_list.attach())
        {
            // 内存不存在，尝试创建
            if (!XYR_memory_list.create(sizeof(double) * 3))
            {
                // create 失败，说明可能存在旧的残留内存，尝试 detach 再 create
                if (XYR_memory_list.error() == QSharedMemory::AlreadyExists)
                {
                    XYR_memory_list.detach();
                    if (!XYR_memory_list.create(sizeof(double) * 3))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to create shared memory: %s", XYR_memory_list.errorString().toStdString().c_str());
                    }
                }
            }

            // 初始化内容
            if (XYR_memory_list.isAttached())
            {
                XYR_memory_list.lock();
                memcpy(XYR_memory_list.data(), direction, sizeof(double) * 3);
                directionArray = static_cast<double *>(XYR_memory_list.data());
                XYR_memory_list.unlock();
            }
        }
        else
        {
            std::cout << "Attached to existing shared memory." << std::endl;
            XYR_memory_list.lock();
            memcpy(XYR_memory_list.data(), direction, sizeof(double) * 3);
            directionArray = static_cast<double *>(XYR_memory_list.data());
            XYR_memory_list.unlock();
        }

        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // Relay panel changes over ROS as well, so RViz may run on another host.
        alignment_subscription = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/map_odom/set", 10,
            [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
                if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
                    !std::isfinite(msg->theta) || !XYR_memory_list.isAttached() ||
                    !XYR_memory_list.lock())
                {
                    return;
                }
                auto *data = static_cast<double *>(XYR_memory_list.data());
                if (data != nullptr)
                {
                    data[0] = msg->x;
                    data[1] = msg->y;
                    data[2] = msg->theta;
                }
                XYR_memory_list.unlock();
            });

        map_to_odom = Eigen::Isometry3d();
        x = 0.88;
        y = -0.816;
        map_to_odom.translation() = Eigen::Vector3d(
            x, y, 0);

        tf2::Quaternion q;
        yaw = 0.0;
        pitch = 0.0;
        roll = 0.0;
        q.setRPY(roll, pitch, yaw);
        map_to_odom.linear() = Eigen::Quaterniond(
                                   q.w(), q.x(), q.y(), q.z())
                                   .toRotationMatrix();
        map_to_odom_publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ModifyMapToOdom::map_to_odom_publish_callback, this));
        listen_keyboard_timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ModifyMapToOdom::listen_keyboard, this));
    }

    ModifyMapToOdom::~ModifyMapToOdom()
    {
        if (XYR_memory_list.isAttached())
        {
            XYR_memory_list.detach();
        }
    }

    void ModifyMapToOdom::map_to_odom_publish_callback()
    {
        if (!XYR_memory_list.isAttached() || !XYR_memory_list.lock())
        {
            return;
        }
        map_to_odom.translation() = Eigen::Vector3d(
            directionArray[0], directionArray[1], 0);
        tf2::Quaternion q;
        q.setRPY(roll, pitch, directionArray[2]);
        map_to_odom.linear() = Eigen::Quaterniond(
                                   q.w(), q.x(), q.y(), q.z())
                                   .toRotationMatrix();
        geometry_msgs::msg::TransformStamped tf_map_to_odom = tf2::eigenToTransform(map_to_odom);
        XYR_memory_list.unlock();
        tf_map_to_odom.header.stamp = this->now();
        tf_map_to_odom.header.frame_id = parent_frame;
        tf_map_to_odom.child_frame_id = target_frame;
        broadcaster->sendTransform(tf_map_to_odom);
    }

    void ModifyMapToOdom::listen_keyboard()
    {

        if (XYR_memory_list.isAttached())
        {

            map_to_odom_lock.lock();
            XYR_memory_list.lock();
            directionArray = static_cast<double *>(XYR_memory_list.data());
            XYR_memory_list.unlock();
            map_to_odom_lock.unlock();
        }
        else
        {
            std::cerr << "Failed to attach shared memory: " << XYR_memory_list.errorString().toStdString() << std::endl;
        }
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(modify_map_to_odom::ModifyMapToOdom)
