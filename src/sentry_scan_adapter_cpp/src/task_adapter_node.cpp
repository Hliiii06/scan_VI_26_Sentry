// 任务适配层（C++）：把任意坐标系的 Mode 1 目标 / Mode 3 路线转换到规划系。
// 与 Python 版 semantics 一致：空 frame 默认拒绝（除非 empty_frame_is_planning）、未知 frame 拒绝、
// 时间戳（未来容差 / 来源年龄）检查、逐点变换、输出 header 改为规划系。
#include <cmath>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace sentry_scan_adapter_cpp {

class TaskAdapter : public rclcpp::Node {
 public:
  TaskAdapter() : Node("task_adapter") {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    goal_in_topic_ = declare_parameter<std::string>("goal_in_topic", "task/goal_in");
    goal_out_topic_ = declare_parameter<std::string>("goal_out_topic", "goal");
    path_in_topic_ = declare_parameter<std::string>("path_in_topic", "task/path_in");
    path_out_topic_ = declare_parameter<std::string>("path_out_topic", "initial_path");
    tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.0);
    tf_future_tolerance_ = declare_parameter<double>("tf_future_tolerance", 0.15);
    max_source_age_ = declare_parameter<double>("max_source_age", 1.0);
    max_future_stamp_ = declare_parameter<double>("max_future_stamp", 0.05);
    empty_frame_is_planning_ = declare_parameter<bool>("empty_frame_is_planning", false);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_out_topic_, 1);
    auto latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_out_topic_, latch);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_in_topic_, 1,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_goal(msg); });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_in_topic_, 1, [this](nav_msgs::msg::Path::SharedPtr msg) { on_path(msg); });

    RCLCPP_WARN(get_logger(),
                "Task adapter (C++) ready: '%s' -> '%s', '%s' -> '%s'; inputs are transformed "
                "into '%s' at their message stamp. Unknown/empty frames are rejected.",
                goal_in_topic_.c_str(), goal_out_topic_.c_str(), path_in_topic_.c_str(),
                path_out_topic_.c_str(), planning_frame_.c_str());
  }

 private:
  static double stamp_s(const std_msgs::msg::Header &header) {
    return static_cast<double>(header.stamp.sec) +
           static_cast<double>(header.stamp.nanosec) * 1e-9;
  }

  bool stamp_ok(const std::string &what, const std_msgs::msg::Header &header) {
    const double stamp = stamp_s(header);
    const double now = get_clock()->now().seconds();
    if (!std::isfinite(stamp) || stamp <= 0.0) {
      RCLCPP_WARN(get_logger(), "%s rejected: missing/invalid header.stamp", what.c_str());
      return false;
    }
    if (stamp > now + max_future_stamp_) {
      RCLCPP_WARN(get_logger(), "%s rejected: stamp %.3f is in the future (limit %.3f)",
                  what.c_str(), stamp, max_future_stamp_);
      return false;
    }
    if ((now - stamp) > max_source_age_) {
      RCLCPP_WARN(get_logger(), "%s rejected: source stamp is %.3fs old (limit %.3fs)",
                  what.c_str(), now - stamp, max_source_age_);
      return false;
    }
    return true;
  }

  std::optional<geometry_msgs::msg::TransformStamped> frame_transform(
      const std::string &what, const std_msgs::msg::Header &header) {
    if (header.frame_id.empty()) {
      if (empty_frame_is_planning_) {
        return std::nullopt;  // 空 frame 视为规划系（调用方不再变换）
      }
      RCLCPP_WARN(get_logger(),
                  "%s rejected: empty header.frame_id (set empty_frame_is_planning only after "
                  "confirming the publisher really uses the planning frame)",
                  what.c_str());
      return std::nullopt;
    }
    if (header.frame_id == planning_frame_) {
      return std::nullopt;
    }
    const double stamp = stamp_s(header);
    try {
      return tf_buffer_->lookupTransform(
          planning_frame_, header.frame_id,
          tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(stamp * 1e9))),
          tf2::durationFromSec(tf_lookup_timeout_));
    } catch (const tf2::TransformException &exc) {
      const std::string message = exc.what();
      if (message.find("extrapolation") != std::string::npos ||
          message.find("future") != std::string::npos) {
        try {
          auto latest = tf_buffer_->lookupTransform(planning_frame_, header.frame_id,
                                                    tf2::TimePointZero,
                                                    tf2::durationFromSec(tf_lookup_timeout_));
          const double delay = stamp - stamp_s(latest.header);
          if (delay >= 0.0 && delay <= tf_future_tolerance_) {
            return latest;
          }
        } catch (const tf2::TransformException &) {
        }
      }
      RCLCPP_WARN(get_logger(),
                  "%s rejected: cannot transform '%s' into '%s' at %.3f (no TF: %s); "
                  "unknown frames are never passed through",
                  what.c_str(), header.frame_id.c_str(), planning_frame_.c_str(), stamp,
                  message.c_str());
      return std::nullopt;
    }
  }

  bool transform_pose(const std::string &what, geometry_msgs::msg::PoseStamped &pose) {
    if (pose.header.frame_id == planning_frame_ ||
        (empty_frame_is_planning_ && pose.header.frame_id.empty())) {
      pose.header.frame_id = planning_frame_;
      return true;
    }
    auto tf = frame_transform(what, pose.header);
    if (!tf) {
      return false;
    }
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(pose, out, tf.value());
    pose = out;
    pose.header.frame_id = planning_frame_;
    return true;
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!stamp_ok("goal", msg->header)) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose = *msg;
    const std::string from = pose.header.frame_id.empty() ? "<empty>" : pose.header.frame_id;
    if (!transform_pose("goal", pose)) {
      return;
    }
    goal_pub_->publish(pose);
    RCLCPP_WARN(get_logger(), "goal %s (%.3f, %.3f, %.3f) -> %s (%.3f, %.3f, %.3f)",
                from.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                planning_frame_.c_str(), pose.pose.position.x, pose.pose.position.y,
                pose.pose.position.z);
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "path rejected: empty path");
      return;
    }
    if (!stamp_ok("path", msg->header)) {
      return;
    }
    nav_msgs::msg::Path out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = planning_frame_;
    for (const auto &input_pose : msg->poses) {
      geometry_msgs::msg::PoseStamped pose = input_pose;
      if (pose.header.frame_id.empty()) {
        pose.header = msg->header;
      }
      if (!transform_pose("path pose", pose)) {
        return;
      }
      out.poses.push_back(pose);
    }
    path_pub_->publish(out);
    RCLCPP_WARN(get_logger(), "path %s (%zu poses) -> %s", msg->header.frame_id.c_str(),
                out.poses.size(), planning_frame_.c_str());
  }

  std::string planning_frame_, goal_in_topic_, goal_out_topic_, path_in_topic_, path_out_topic_;
  double tf_lookup_timeout_{}, tf_future_tolerance_{}, max_source_age_{}, max_future_stamp_{};
  bool empty_frame_is_planning_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sentry_scan_adapter_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_scan_adapter_cpp::TaskAdapter>());
  rclcpp::shutdown();
  return 0;
}
