#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>

namespace uav_rviz_plugins {

class MapOdomPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit MapOdomPanel(QWidget *parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

private:
  void connectTopics();
  void publish();
  void shift(double dx, double dy, double dyaw);

  QLineEdit *namespace_edit_;
  QDoubleSpinBox *x_, *y_, *yaw_;
  QLabel *status_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_;
};

}  // namespace uav_rviz_plugins
