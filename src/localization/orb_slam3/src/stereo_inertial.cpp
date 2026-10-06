#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include "System.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>

namespace {
double stamp(const builtin_interfaces::msg::Time &t) { return t.sec + 1e-9 * t.nanosec; }
struct ImuSample {
  double t;
  Eigen::Vector3f accel, gyro;
};
}

class StereoInertial : public rclcpp::Node {
public:
  StereoInertial() : Node("orbslam") {
    const auto settings = declare_parameter<std::string>("settings_file", "");
    const auto vocabulary = declare_parameter<std::string>("vocabulary_file", "");
    const auto rectify = declare_parameter<std::string>("rectification_file", "");
    const bool viewer = declare_parameter<bool>("viewer", false);
    offset_ = declare_parameter<double>("camera_imu_timeshift", 0.0);
    slop_ = declare_parameter<double>("stereo_sync_max_interval", .003);
    image_limit_ = declare_parameter<int>("image_queue_size", 12);
    imu_limit_ = declare_parameter<int>("imu_queue_size", 4000);
    max_imu_gap_ = declare_parameter<double>("max_imu_gap", .03);
    max_image_age_ = declare_parameter<double>("max_image_age", .3);
    future_tolerance_ = declare_parameter<double>("future_tolerance", .02);
    min_inliers_ = declare_parameter<int>("min_inliers", 30);
    pv_ = declare_parameter<double>("position_variance", .04);
    ov_ = declare_parameter<double>("orientation_variance", .01);
    vv_ = declare_parameter<double>("velocity_variance", .1);
    if(settings.empty() || vocabulary.empty() || rectify.empty() ||
       image_limit_ < 2 || imu_limit_ < 10 || min_inliers_ < 1 ||
       !(slop_ > 0 && max_imu_gap_ > 0 && max_image_age_ > 0 &&
         future_tolerance_ >= 0 && pv_ > 0 && ov_ > 0 && vv_ > 0) ||
       !std::isfinite(offset_)) throw std::runtime_error("invalid ORB configuration");
    cv::FileStorage fs(rectify, cv::FileStorage::READ);
    fs["left_map_x"] >> lx_; fs["left_map_y"] >> ly_;
    fs["right_map_x"] >> rx_; fs["right_map_y"] >> ry_;
    if(lx_.empty() || ly_.empty() || rx_.empty() || ry_.empty())
      throw std::runtime_error("missing measured stereo rectification maps");
    orb_ = std::make_unique<ORB_SLAM3::System>(vocabulary, settings,
                                              ORB_SLAM3::System::IMU_STEREO, viewer);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odomimu", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/orbslam/tracking_state", 10);
    reset_pub_ = create_publisher<std_msgs::msg::Header>("/estimator_reset",
        rclcpp::QoS(1).reliable().transient_local());
    const auto qos = rclcpp::SensorDataQoS().keep_last(20);
    left_sub_ = create_subscription<sensor_msgs::msg::Image>("cam0/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { image(m, left_, left_stamp_); });
    right_sub_ = create_subscription<sensor_msgs::msg::Image>("cam1/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { image(m, right_, right_stamp_); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu0", qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu(m); });
    epoch(now().seconds(), "START");
    worker_ = std::thread([this] { run(); });
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 ready; calibrated stereo + IMU, viewer=%d", viewer);
  }

  ~StereoInertial() override {
    stop_ = true; cv_.notify_all();
    if(worker_.joinable()) worker_.join();
    if(orb_) orb_->Shutdown();
  }

private:
  using Image = sensor_msgs::msg::Image;
  void status(const std::string &text) {
    std_msgs::msg::String m; m.data = text; state_pub_->publish(m);
  }
  void clear_locked(const std::string &reason) {
    left_.clear(); right_.clear(); imu_.clear();
    left_stamp_ = right_stamp_ = imu_stamp_ = -1;
    reset_reason_ = reason;
  }
  void image(Image::ConstSharedPtr m, std::deque<Image::ConstSharedPtr> &q, double &last) {
    const double t = stamp(m->header.stamp);
    if(!std::isfinite(t) || t <= 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if(t < last) clear_locked("IMAGE_STAMP_REGRESSION");
    if(t == last) return;
    last = t; q.push_back(m);
    while(q.size() > static_cast<size_t>(image_limit_)) { q.pop_front(); ++dropped_; }
    cv_.notify_all();
  }
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr m) {
    ImuSample s{stamp(m->header.stamp),
      Eigen::Vector3f(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z),
      Eigen::Vector3f(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z)};
    if(!std::isfinite(s.t) || s.t <= 0 || !s.accel.allFinite() || !s.gyro.allFinite()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if(s.t < imu_stamp_) clear_locked("IMU_STAMP_REGRESSION");
    if(s.t == imu_stamp_) return;
    imu_stamp_ = s.t; imu_.push_back(s);
    if(imu_.size() > static_cast<size_t>(imu_limit_)) clear_locked("IMU_QUEUE_OVERFLOW");
    cv_.notify_all();
  }
  void epoch(double t, const std::string &reason) {
    std_msgs::msg::Header msg;
    msg.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    msg.frame_id = "ORB:" + process_id_ + ":" + std::to_string(++epoch_) + ":" + reason;
    reset_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "Estimator epoch: %s", msg.frame_id.c_str());
  }
  void run() {
    double last_frame = -1;
    int last_map = -1, last_correction = -1;
    bool was_ready = false;
    while(!stop_) {
      Image::ConstSharedPtr left, right;
      std::vector<ORB_SLAM3::IMU::Point> readings;
      ImuSample endpoint;
      double t = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(100));
        if(stop_) break;
        if(!reset_reason_.empty()) {
          auto reason = reset_reason_; reset_reason_.clear();
          lock.unlock(); orb_->ResetActiveMap();
          last_frame = -1; last_map = last_correction = -1; was_ready = false; epoch(now().seconds(), reason); status(reason);
          continue;
        }
        while(!left_.empty() && !right_.empty()) {
          double dt = stamp(left_.front()->header.stamp) - stamp(right_.front()->header.stamp);
          if(std::abs(dt) <= slop_) break;
          if(dt < 0) left_.pop_front(); else right_.pop_front();
          ++dropped_;
        }
        if(left_.empty() || right_.empty() || imu_.size() < 2) continue;
        t = stamp(left_.front()->header.stamp) + offset_;
        double age = now().seconds() - t;
        if(age > max_image_age_ || age < -future_tolerance_ || t <= last_frame) {
          left_.pop_front(); right_.pop_front(); ++dropped_; status("IMAGE_STALE"); continue;
        }
        if(imu_.back().t < t) continue; // wait for the upper interpolation sample
        const double begin = last_frame > 0 ? last_frame : t - .02;
        if(last_frame > 0 && imu_.front().t > begin + max_imu_gap_) {
          clear_locked("IMU_COVERAGE_GAP"); continue;
        }
        while(imu_.size() > 2 && imu_[1].t < begin) imu_.pop_front();
        bool gap = false;
        size_t upper = 0;
        while(upper < imu_.size() && imu_[upper].t < t) ++upper;
        if(upper == 0 || upper >= imu_.size()) {
          left_.pop_front(); right_.pop_front(); ++dropped_; status("IMU_NOT_BRACKETED"); continue;
        }
        for(size_t i = 1; i <= upper; ++i)
          if(imu_[i].t - imu_[i-1].t > max_imu_gap_) gap = true;
        if(gap) { clear_locked("IMU_SAMPLE_GAP"); continue; }
        const auto &a = imu_[upper-1]; const auto &b = imu_[upper];
        const float alpha = static_cast<float>((t-a.t)/(b.t-a.t));
        endpoint = {t, a.accel + alpha*(b.accel-a.accel), a.gyro + alpha*(b.gyro-a.gyro)};
        for(size_t i = 0; i < upper; ++i)
          if(imu_[i].t > begin)
            readings.emplace_back(imu_[i].accel.x(), imu_[i].accel.y(), imu_[i].accel.z(),
                                  imu_[i].gyro.x(), imu_[i].gyro.y(), imu_[i].gyro.z(), imu_[i].t);
        readings.emplace_back(endpoint.accel.x(), endpoint.accel.y(), endpoint.accel.z(),
                              endpoint.gyro.x(), endpoint.gyro.y(), endpoint.gyro.z(), t);
        left = left_.front(); right = right_.front(); left_.pop_front(); right_.pop_front();
      }
      try {
        auto l = cv_bridge::toCvCopy(left, "mono8")->image;
        auto r = cv_bridge::toCvCopy(right, "mono8")->image;
        if(l.size() != lx_.size() || r.size() != rx_.size())
          throw std::runtime_error("image dimensions differ from measured calibration");
        cv::Mat lr, rr;
        cv::remap(l, lr, lx_, ly_, cv::INTER_LINEAR);
        cv::remap(r, rr, rx_, ry_, cv::INTER_LINEAR);
        orb_->TrackStereo(lr, rr, t, readings);
        last_frame = t;
        Sophus::SE3f Twi; Eigen::Vector3f v; ORB_SLAM3::IMU::Bias bias;
        int map_id = -1, inliers = 0, correction = 0;
        bool ready = orb_->GetCurrentImuState(Twi, v, bias, map_id, inliers, correction);
        const bool new_map = last_map >= 0 && map_id != last_map;
        const bool corrected = !new_map && last_correction >= 0 && correction != last_correction;
        if(new_map) epoch(t, "MAP_RECREATED");
        else if(corrected) epoch(t, "MAP_CORRECTION");
        else if(ready && !was_ready) epoch(t, "INERTIAL_TRACKING_READY");
        // Tracking recovery alone does not necessarily change coordinates, so only
        // the initial transition in each map announces readiness as a new epoch.
        if(new_map) was_ready = false;
        if(ready) was_ready = true;
        last_map = map_id; last_correction = correction;
        const int state = orb_->GetTrackingState();
        if(!ready || inliers < min_inliers_) {
          const std::string reason = state == ORB_SLAM3::Tracking::OK ?
              (ready ? "LOW_INLIERS" : "IMU_INITIALIZING") :
              state == ORB_SLAM3::Tracking::RECENTLY_LOST ? "RECENTLY_LOST" : "LOST_OR_INITIALIZING";
          status(reason);
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Odometry paused: %s, inliers=%d, map=%d correction=%d",
              reason.c_str(), inliers, map_id, correction);
          continue;
        }
        if(!Twi.matrix().allFinite() || !v.allFinite()) { status("NONFINITE_POSE"); continue; }
        if(now().seconds() - t > max_image_age_) { status("PROCESSING_LATE"); continue; }
        nav_msgs::msg::Odometry out;
        out.header.stamp = rclcpp::Time(static_cast<int64_t>(t*1e9));
        out.header.frame_id = "global"; out.child_frame_id = "imu";
        const auto p = Twi.translation(); const Eigen::Quaternionf q(Twi.rotationMatrix());
        out.pose.pose.position.x=p.x(); out.pose.pose.position.y=p.y(); out.pose.pose.position.z=p.z();
        out.pose.pose.orientation.w=q.w(); out.pose.pose.orientation.x=q.x();
        out.pose.pose.orientation.y=q.y(); out.pose.pose.orientation.z=q.z();
        const Eigen::Vector3f vi = Twi.rotationMatrix().transpose()*v;
        const Eigen::Vector3f omega = endpoint.gyro - Eigen::Vector3f(bias.bwx,bias.bwy,bias.bwz);
        out.twist.twist.linear.x=vi.x(); out.twist.twist.linear.y=vi.y(); out.twist.twist.linear.z=vi.z();
        out.twist.twist.angular.x=omega.x(); out.twist.twist.angular.y=omega.y(); out.twist.twist.angular.z=omega.z();
        // Explicit configurable proxies: upstream does not export estimator covariance.
        for(int i=0;i<3;++i) {
          out.pose.covariance[i*6+i]=pv_; out.pose.covariance[(i+3)*6+i+3]=ov_;
          out.twist.covariance[i*6+i]=vv_; out.twist.covariance[(i+3)*6+i+3]=ov_;
        }
        odom_pub_->publish(out); status("TRACKING");
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                            "tracking inliers=%d dropped_images=%lu map=%d", inliers, dropped_.load(), map_id);
      } catch(const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Tracking rejected: %s", e.what()); status("TRACKING_EXCEPTION");
        std::lock_guard<std::mutex> lock(mutex_); clear_locked("TRACKING_EXCEPTION");
      }
    }
  }
  std::unique_ptr<ORB_SLAM3::System> orb_;
  std::deque<Image::ConstSharedPtr> left_,right_;
  std::deque<ImuSample> imu_;
  std::mutex mutex_; std::condition_variable cv_; std::thread worker_;
  std::atomic<bool> stop_{false}; std::atomic<unsigned long> dropped_{0};
  double left_stamp_=-1,right_stamp_=-1,imu_stamp_=-1;
  double offset_,slop_,max_imu_gap_,max_image_age_,future_tolerance_,pv_,ov_,vv_;
  int image_limit_,imu_limit_,min_inliers_; unsigned long epoch_=0;
  const std::string process_id_ = std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
  std::string reset_reason_;
  cv::Mat lx_,ly_,rx_,ry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr reset_pub_;
  rclcpp::Subscription<Image>::SharedPtr left_sub_,right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<StereoInertial>()); }
  catch(const std::exception &e) { fprintf(stderr,"ORB startup failed: %s\n",e.what()); if(rclcpp::ok()) rclcpp::shutdown(); return 1; }
  if(rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
