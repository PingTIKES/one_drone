#include "uav_rviz_plugins/map_odom_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include <QFormLayout>
#include <QGridLayout>
#include <QMetaObject>
#include <QPushButton>
#include <QVBoxLayout>

#include <cmath>

namespace uav_rviz_plugins {

MapOdomPanel::MapOdomPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QVBoxLayout(this);
  namespace_edit_ = new QLineEdit("/", this);
  x_ = new QDoubleSpinBox(this);
  y_ = new QDoubleSpinBox(this);
  yaw_ = new QDoubleSpinBox(this);
  for (auto *box : {x_, y_}) {
    box->setRange(-100., 100.);
    box->setDecimals(3);
    box->setSingleStep(0.05);
  }
  yaw_->setRange(-180., 180.);
  yaw_->setDecimals(2);
  yaw_->setSingleStep(1.);
  auto *form = new QFormLayout();
  form->addRow("Namespace", namespace_edit_);
  form->addRow("Map → odom X (m)", x_);
  form->addRow("Map → odom Y (m)", y_);
  form->addRow("Yaw (deg)", yaw_);
  layout->addLayout(form);
  auto *apply = new QPushButton("Apply alignment", this);
  layout->addWidget(apply);
  auto *buttons = new QGridLayout();
  auto add = [this, buttons](const char *label, int row, int col,
                              double dx, double dy, double dyaw) {
    auto *button = new QPushButton(label, this);
    buttons->addWidget(button, row, col);
    connect(button, &QPushButton::clicked, this,
            [this, dx, dy, dyaw]() { shift(dx, dy, dyaw); });
  };
  add("↑", 0, 1, 0., .05, 0.);
  add("←", 1, 0, -.05, 0., 0.);
  add("→", 1, 2, .05, 0., 0.);
  add("↓", 2, 1, 0., -.05, 0.);
  add("↶", 1, 1, 0., 0., 1.);
  add("↷", 2, 2, 0., 0., -1.);
  layout->addLayout(buttons);
  status_ = new QLabel("Waiting for map → odom", this);
  layout->addWidget(status_);
  layout->addStretch();
  connect(apply, &QPushButton::clicked, this, [this]() { publish(); });
  connect(namespace_edit_, &QLineEdit::editingFinished, this,
          [this]() { connectTopics(); });
}

void MapOdomPanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_->setText("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  connectTopics();
}

void MapOdomPanel::connectTopics() {
  if (!node_) return;
  QString ns = namespace_edit_->text().trimmed();
  if (!ns.startsWith('/')) ns.prepend('/');
  while (ns.length() > 1 && ns.endsWith('/')) ns.chop(1);
  if (ns.isEmpty()) ns = "/";
  namespace_edit_->setText(ns);
  const std::string prefix = ns == "/" ? "" : ns.toStdString();
  subscriber_.reset();
  publisher_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
      prefix + "/map_odom/set", rclcpp::QoS(10));
  subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
      prefix + "/map_odom/current", rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::Pose2D::SharedPtr msg) {
        const double x = msg->x, y = msg->y, yaw = msg->theta * 180. / M_PI;
        QMetaObject::invokeMethod(this, [this, x, y, yaw]() {
          x_->setValue(x);
          y_->setValue(y);
          yaw_->setValue(yaw);
          status_->setText("Current alignment received");
        }, Qt::QueuedConnection);
      });
  status_->setText("Waiting for " + ns + "/map_odom/current");
}

void MapOdomPanel::publish() {
  if (!publisher_) {
    status_->setText("Map alignment publisher unavailable");
    return;
  }
  geometry_msgs::msg::Pose2D msg;
  msg.x = x_->value();
  msg.y = y_->value();
  msg.theta = yaw_->value() * M_PI / 180.;
  publisher_->publish(msg);
  status_->setText("Alignment sent");
}

void MapOdomPanel::shift(double dx, double dy, double dyaw) {
  x_->setValue(x_->value() + dx);
  y_->setValue(y_->value() + dy);
  yaw_->setValue(yaw_->value() + dyaw);
  publish();
}

void MapOdomPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("Namespace", &ns)) namespace_edit_->setText(ns);
}

void MapOdomPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", namespace_edit_->text());
}

}  // namespace uav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(uav_rviz_plugins::MapOdomPanel, rviz_common::Panel)
