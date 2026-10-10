// Mode 3 参考路线发布器（C++）：等价于原 Python 版 reference_path_publisher.py。
// 语义一致：参数 waypoints（x,y,z 展开的 double 数组）、frame_id、publish_delay_sec；
// 等 odom/body_pose 出现后再周期（1 Hz）发布 initial_path（地面 z，body_height 由 SCAN 加一次），
// 保证后来启动的 FSM 也能拿到路线（transient_local）。
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sentry_scan_adapter_cpp {

class ReferencePathPublisher : public rclcpp::Node {
 public:
  ReferencePathPublisher() : Node("reference_path_publisher") {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    waypoints_ = declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
    publish_delay_sec_ = declare_parameter<double>("publish_delay_sec", 0.5);
    publish_period_sec_ = declare_parameter<double>("publish_period_sec", 1.0);
    start_s_ = now_s();

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("initial_path", path_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", 10, [this](nav_msgs::msg::Odometry::SharedPtr) { odom_seen_ = true; });

    if (waypoints_.size() < 3 || waypoints_.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(),
                   "waypoints must be x,y,z triplets (got %zu values); nothing will be published",
                   waypoints_.size());
      return;
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_sec_),
                               [this]() { try_publish(); });
    RCLCPP_WARN(get_logger(), "Reference path publisher (C++): %zu waypoints in frame '%s'",
                waypoints_.size() / 3, frame_id_.c_str());
  }

 private:
  void try_publish() {
    if (!odom_seen_ || (now_s() - start_s_) < publish_delay_sec_) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = get_clock()->now();
    path.header.frame_id = frame_id_;
    for (std::size_t index = 0; index + 2 < waypoints_.size(); index += 3) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoints_[index];
      pose.pose.position.y = waypoints_[index + 1];
      pose.pose.position.z = waypoints_[index + 2];
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  double now_s() { return get_clock()->now().seconds(); }

  std::string frame_id_;
  std::vector<double> waypoints_;
  double publish_delay_sec_{}, publish_period_sec_{};
  bool odom_seen_{false};
  double start_s_{0.0};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_scan_adapter_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_scan_adapter_cpp::ReferencePathPublisher>());
  rclcpp::shutdown();
  return 0;
}
