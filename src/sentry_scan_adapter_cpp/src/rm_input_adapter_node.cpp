// C++ 运行适配层：实车输入（里程计/速度/点云）-> SCAN 规划系话题。
//
// 与 Python 影子适配层（sentry_scan_adapter/rm_input_adapter.py）**语义一致**，但：
//   * 点云用 PCL 读取（`pcl::fromROSMsg`），现场 48 字节 PCL 风格布局、行填充、字段空洞都能读；
//     不再依赖 tf2_sensor_msgs 的 Python 实现（它会对这种 dtype 报错）；
//   * 常驻车上运行，无 numpy/rclpy 依赖。
//
// 关键语义（与 Python 版逐条对应，均已在合成场景里验证过）：
//   1) 里程计若已在规划系（header.frame_id == planning_frame）→ 直接用消息位姿，
//      child_frame_id 与 body_frame 的差异用**静态** TF 修正（实车 TfTransformer 先发 odom
//      后广播同 stamp 的动态 TF，按消息时刻查会拿到上一周期样本而被拒）；
//   2) 速度源 frame（实车是 world）没有 TF 且未声明 alias → 线/角速度置零并给出原因（不算不健康）；
//   3) 每条通道都做：时间戳（未来容差、倒退、来源年龄、接收年龄）、有限性、参考点/配对检查；
//   4) 健康 = 所有通道都有有效样本；不健康时发 planning/reset，恢复后不自动续跑旧任务。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace sentry_scan_adapter_cpp {

// 点云布局校验（与 Python 版逐条对应）：不能只靠 pcl::fromROSMsg —— PCL 对缺少 xyz 字段的
// 消息只打印警告并返回 (0,0,0) 点，足以骗过"有限点数"检查。先校验再转换。
inline std::string validate_cloud_layout(const sensor_msgs::msg::PointCloud2 &msg) {
  if (msg.is_bigendian) {
    return "unsupported layout: is_bigendian=true";
  }
  if (msg.width == 0 || msg.height == 0) {
    return "empty cloud";
  }
  if (msg.point_step < 12) {
    return "point_step=" + std::to_string(msg.point_step) + " < 12";
  }
  if (msg.row_step < msg.width * msg.point_step) {
    return "row_step < width*point_step";
  }
  if (static_cast<std::size_t>(msg.height) * msg.row_step > msg.data.size()) {
    return "data shorter than height*row_step (truncated)";
  }
  std::map<std::string, const sensor_msgs::msg::PointField *> found;
  for (const auto &field : msg.fields) {
    if (field.name != "x" && field.name != "y" && field.name != "z") {
      continue;
    }
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      return "field " + field.name + " is not FLOAT32";
    }
    if (field.count != 1) {
      return "field " + field.name + " count != 1";
    }
    if (static_cast<int>(field.offset) + 4 > static_cast<int>(msg.point_step)) {
      return "field " + field.name + " offset exceeds point_step";
    }
    found[field.name] = &field;
  }
  if (found.size() != 3) {
    return "missing x/y/z fields";
  }
  return "";
}

struct Channel {
  std::string name;
  long count = 0;
  long rejected = 0;
  double last_receive = 0.0;
  double last_valid = 0.0;
  std::string last_frame;
  std::string error;
  // 时间戳历史
  double last_stamp = 0.0;
  bool has_stamp = false;
};

class RmInputAdapter : public rclcpp::Node {
 public:
  RmInputAdapter() : Node("rm_input_adapter") {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "lidar_link");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry_transformed");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "/LIVO2/imu_propagate");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    velocity_frame_ = declare_parameter<std::string>("velocity_frame", "world");
    velocity_frame_alias_ = declare_parameter<std::string>("velocity_frame_alias", "");
    // YAML 里是字符串（与 Python 版一致）：留空 = 不补偿；格式 "x,y,z"
    body_center_offset_ = parse_offsets(
        declare_parameter<std::string>("body_center_offset_xyz", ""));
    imu_to_body_offset_ = parse_offsets(
        declare_parameter<std::string>("imu_to_body_offset_xyz", ""));
    max_source_age_ = declare_parameter<double>("max_source_age", 0.5);
    max_receive_age_ = declare_parameter<double>("max_receive_age", 0.5);
    max_stamp_regression_ = declare_parameter<double>("max_stamp_regression", 0.0);
    max_future_stamp_ = declare_parameter<double>("max_future_stamp", 0.05);
    min_valid_points_ = declare_parameter<int>("min_valid_points", 10);
    tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.0);
    tf_future_tolerance_ = declare_parameter<double>("tf_future_tolerance", 0.15);
    odom_in_planning_frame_ = declare_parameter<bool>("odom_in_planning_frame", true);
    require_center_velocity_ = declare_parameter<bool>("require_center_velocity", false);
    require_angular_velocity_ = declare_parameter<bool>("require_angular_velocity", false);
    cloud_assume_planning_frame_ = declare_parameter<bool>("cloud_assume_planning_frame", false);
    health_period_ = declare_parameter<double>("health_period", 0.5);
    reset_on_unhealthy_ = declare_parameter<bool>("reset_on_unhealthy", true);
    reset_repeat_period_ = declare_parameter<double>("reset_repeat_period", 1.0);
    jump_translation_ = declare_parameter<double>("localization_jump_translation", 0.5);
    jump_rotation_ = declare_parameter<double>("localization_jump_rotation", 0.35);
    jump_latch_duration_ = declare_parameter<double>("jump_latch_duration", 0.0);

    channels_["odom"].name = "odom";
    channels_["velocity"].name = "velocity";
    channels_["cloud"].name = "cloud";
    channels_["tf"].name = "tf";

    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });
    velocity_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        velocity_topic_, sensor_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_velocity(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cloud(msg); });

    body_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 10);
    sensor_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensor_pose", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("health", 10);
    auto latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    health_ok_pub_ = create_publisher<std_msgs::msg::Bool>("health_ok", latch);
    reset_pub_ = create_publisher<std_msgs::msg::Bool>("planning/reset", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    health_timer_ = create_wall_timer(
        std::chrono::duration<double>(health_period_), [this]() { publish_health(); });

    RCLCPP_WARN(get_logger(),
                "RM input adapter (C++, SHADOW/TAKEOVER, read-only inputs). planning=%s task=n/a "
                "body=%s sensor=%s cloud_assume_planning_frame=%s source_age<=%.2fs "
                "receive_age<=%.2fs max_future_stamp=%.3fs min_valid_points=%d",
                planning_frame_.c_str(), body_frame_.c_str(), sensor_frame_.c_str(),
                cloud_assume_planning_frame_ ? "true" : "false", max_source_age_,
                max_receive_age_, max_future_stamp_, min_valid_points_);
    RCLCPP_WARN(get_logger(), "This node publishes NO velocity command.");
  }

 private:
  // ------------------------------------------------------------------ 工具
  double now_s() { return get_clock()->now().seconds(); }

  static std::vector<double> parse_offsets(const std::string &text) {
    std::vector<double> out;
    std::string current;
    for (const char c : text) {
      if (c == ',' || c == ' ' || c == ';') {
        if (!current.empty()) {
          try {
            out.push_back(std::stod(current));
          } catch (const std::exception &) {
            return {};
          }
          current.clear();
        }
      } else {
        current.push_back(c);
      }
    }
    if (!current.empty()) {
      try {
        out.push_back(std::stod(current));
      } catch (const std::exception &) {
        return {};
      }
    }
    return out;
  }

  static double stamp_s(const std_msgs::msg::Header &header) {
    return static_cast<double>(header.stamp.sec) +
           static_cast<double>(header.stamp.nanosec) * 1e-9;
  }

  static bool finite3(double x, double y, double z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  void reject(Channel &channel, const std::string &reason) {
    channel.rejected++;
    channel.error = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s rejected: %s",
                         channel.name.c_str(), reason.c_str());
  }

  std::optional<double> accept_stamp(Channel &channel, double stamp, double recv) {
    if (!std::isfinite(stamp) || stamp <= 0.0) {
      reject(channel, "missing/invalid header.stamp");
      return std::nullopt;
    }
    if (stamp > recv + max_future_stamp_) {
      reject(channel, "timestamp is in the future beyond tolerance; not written to history");
      return std::nullopt;
    }
    if (channel.has_stamp && stamp < channel.last_stamp - max_stamp_regression_) {
      reject(channel, "timestamp went backwards");
      return std::nullopt;
    }
    if ((recv - stamp) > max_source_age_) {
      reject(channel, "source stamp too old");
      return std::nullopt;
    }
    channel.last_stamp = stamp;
    channel.has_stamp = true;
    return stamp;
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookup(
      const std::string &target, const std::string &source, double stamp, bool allow_future,
      bool required = true) {
    geometry_msgs::msg::TransformStamped tf;
    double delay = 0.0;
    try {
      tf = tf_buffer_->lookupTransform(target, source, tf2::TimePoint(
          std::chrono::nanoseconds(static_cast<int64_t>(stamp * 1e9))),
          tf2::durationFromSec(tf_lookup_timeout_));
    } catch (const tf2::TransformException &exc) {
      const std::string message = exc.what();
      const bool extrapolation = message.find("extrapolation") != std::string::npos ||
                                 message.find("future") != std::string::npos;
      if (allow_future && extrapolation) {
        try {
          tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero,
                                           tf2::durationFromSec(tf_lookup_timeout_));
          const double latest = stamp_s(tf.header);
          delay = stamp - latest;
          if (delay < 0.0 || delay > tf_future_tolerance_) {
            if (required) {
              reject(channels_["tf"],
                     "lookup " + target + "<-" + source + " needs future data beyond tolerance");
            }
            return std::nullopt;
          }
        } catch (const tf2::TransformException &exc2) {
          if (required) {
            reject(channels_["tf"], std::string("lookup failed: ") + message +
                                        " / fallback: " + exc2.what());
          }
          return std::nullopt;
        }
      } else {
        if (required) {
          reject(channels_["tf"], "lookup " + target + "<-" + source + " failed: " + message);
        }
        return std::nullopt;
      }
    }
    Channel &tf_channel = channels_["tf"];
    tf_channel.count++;
    tf_channel.last_valid = now_s();
    tf_channel.last_receive = tf_channel.last_valid;
    tf_channel.last_frame = target + "<-" + source;
    if (!jump_latched_) {
      tf_channel.error.clear();
    }
    return tf;
  }

  // ------------------------------------------------------------------ 里程计
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Channel &channel = channels_["odom"];
    channel.count++;
    const double recv = now_s();
    const double stamp = stamp_s(msg->header);
    if (!accept_stamp(channel, stamp, recv)) {
      return;
    }
    const auto &pose_in = msg->pose.pose;
    if (!finite3(pose_in.position.x, pose_in.position.y, pose_in.position.z)) {
      reject(channel, "non-finite position");
      return;
    }

    geometry_msgs::msg::Pose body_pose;
    if (odom_in_planning_frame_ && msg->header.frame_id == planning_frame_) {
      body_pose = pose_in;
      const std::string child =
          msg->child_frame_id.empty() ? body_frame_ : msg->child_frame_id;
      if (child != body_frame_) {
        // 同一机体的不同参考点：用静态 TF（TimePointZero，与发布顺序无关）修正
        auto offset = lookup(body_frame_, child, 0.0, false, false);
        if (offset) {
          geometry_msgs::msg::Pose out;
          tf2::doTransform(body_pose, out, offset.value());
          body_pose = out;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "static %s<-%s unavailable; using the message pose as-is",
                               body_frame_.c_str(), child.c_str());
        }
      }
      channel.last_valid = recv;
      channel.last_receive = recv;
      channel.last_frame = planning_frame_;
      channel.error.clear();
      emit_body_pose(body_pose, stamp, "message");
      return;
    }

    auto tf = lookup(planning_frame_, body_frame_, stamp, true);
    if (!tf) {
      return;
    }
    geometry_msgs::msg::Pose out;
    tf2::doTransform(pose_in, out, tf.value());
    channel.last_valid = recv;
    channel.last_receive = recv;
    channel.last_frame = planning_frame_;
    channel.error.clear();
    emit_body_pose(out, stamp, "tf");
  }

  void emit_body_pose(const geometry_msgs::msg::Pose &pose_in, double stamp,
                      const std::string &source) {
    geometry_msgs::msg::Pose pose = pose_in;
    if (body_center_offset_.size() == 3 &&
        (body_center_offset_[0] != 0.0 || body_center_offset_[1] != 0.0 ||
         body_center_offset_[2] != 0.0)) {
      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      tf2::Vector3 offset(body_center_offset_[0], body_center_offset_[1],
                          body_center_offset_[2]);
      tf2::Vector3 rotated = tf2::quatRotate(q, offset);
      pose.position.x += rotated.x();
      pose.position.y += rotated.y();
      pose.position.z += rotated.z();
    }

    auto [linear, angular, reason] = build_velocity(stamp);

    nav_msgs::msg::Odometry out;
    out.header.stamp.sec = static_cast<int32_t>(std::floor(stamp));
    out.header.stamp.nanosec = static_cast<uint32_t>((stamp - std::floor(stamp)) * 1e9);
    out.header.frame_id = planning_frame_;
    out.child_frame_id = body_frame_;
    out.pose.pose = pose;
    out.twist.twist.linear.x = linear[0];
    out.twist.twist.linear.y = linear[1];
    out.twist.twist.linear.z = linear[2];
    out.twist.twist.angular.x = angular[0];
    out.twist.twist.angular.y = angular[1];
    out.twist.twist.angular.z = angular[2];
    channels_["odom"].error.clear();
    velocity_reason_ = reason;
    body_pose_pub_->publish(out);
    (void)source;
  }

  // ------------------------------------------------------------------ 速度
  void on_velocity(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Channel &channel = channels_["velocity"];
    channel.count++;
    const double recv = now_s();
    const double stamp = stamp_s(msg->header);
    if (!accept_stamp(channel, stamp, recv)) {
      return;
    }
    const auto &twist = msg->twist.twist;
    if (!finite3(twist.linear.x, twist.linear.y, twist.linear.z) ||
        !finite3(twist.angular.x, twist.angular.y, twist.angular.z)) {
      reject(channel, "non-finite velocity");
      return;
    }
    velocity_msg_ = *msg;
    velocity_valid_ = true;
    velocity_stamp_ = stamp;
    channel.last_valid = recv;
    channel.last_receive = recv;
    channel.last_frame = msg->header.frame_id;
    channel.error.clear();
  }

  std::tuple<std::array<double, 3>, std::array<double, 3>, std::string> build_velocity(
      double stamp) {
    std::array<double, 3> zero{0.0, 0.0, 0.0};
    if (!velocity_valid_) {
      return {zero, zero, "velocity_missing"};
    }
    const double age = std::fabs(stamp - velocity_stamp_);
    if (age > max_source_age_) {
      return {zero, zero, "velocity_stale_stamp"};
    }
    std::string frame = velocity_msg_.header.frame_id;
    if (!velocity_frame_alias_.empty() && frame == velocity_frame_) {
      frame = velocity_frame_alias_;
    }
    geometry_msgs::msg::TransformStamped tf;
    if (frame == planning_frame_) {
      // 已是规划系
    } else {
      auto found = lookup(planning_frame_, frame, velocity_stamp_, true, false);
      if (!found) {
        return {zero, zero, "velocity_frame_unresolved(" + velocity_msg_.header.frame_id + ")"};
      }
      tf = found.value();
    }
    const auto &twist = velocity_msg_.twist.twist;
    tf2::Vector3 linear(twist.linear.x, twist.linear.y, twist.linear.z);
    tf2::Vector3 angular(twist.angular.x, twist.angular.y, twist.angular.z);
    if (frame != planning_frame_) {
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      linear = tf2::quatRotate(q, linear);
      angular = tf2::quatRotate(q, angular);
    }
    if (imu_to_body_offset_.size() != 3 && require_center_velocity_) {
      return {zero, zero, "lever_arm_uncompensated"};
    }
    if (require_angular_velocity_ && angular.length() < 1e-9) {
      return {zero, zero, "angular_velocity_missing"};
    }
    return {{linear.x(), linear.y(), linear.z()}, {angular.x(), angular.y(), angular.z()}, ""};
  }

  // ------------------------------------------------------------------ 点云
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Channel &channel = channels_["cloud"];
    channel.count++;
    const double recv = now_s();
    const double stamp = stamp_s(msg->header);
    if (!accept_stamp(channel, stamp, recv)) {
      return;
    }
    if (msg->width * msg->height == 0) {
      reject(channel, "empty point cloud");
      return;
    }
    // 先做布局校验（缺字段/错类型/截断/大端一律拒绝），再交给 PCL 解析。
    // 现场为 48 字节 PCL 风格（x@0 y@4 z@8 normal_*@16..24 intensity@32 curvature@36），
    // 字段间有空洞是允许的；这里只拒绝"不可信"的布局。
    const std::string layout_error = validate_cloud_layout(*msg);
    if (!layout_error.empty()) {
      reject(channel, "point cloud layout unsupported: " + layout_error);
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    try {
      pcl::fromROSMsg(*msg, cloud);
    } catch (const std::exception &exc) {
      reject(channel, std::string("pcl::fromROSMsg failed: ") + exc.what());
      return;
    }
    std::size_t finite = 0;
    for (const auto &point : cloud.points) {
      if (pcl::isFinite(point)) {
        ++finite;
      }
    }
    if (static_cast<int>(finite) < min_valid_points_) {
      reject(channel, "point cloud has " + std::to_string(finite) +
                          " finite xyz points (< " + std::to_string(min_valid_points_) + ")");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> transformed;
    if (msg->header.frame_id == planning_frame_ ||
        (cloud_assume_planning_frame_ && msg->header.frame_id.empty())) {
      transformed = cloud;
    } else {
      auto tf = lookup(planning_frame_, msg->header.frame_id, stamp, true);
      if (!tf) {
        return;
      }
      // 手工构造 Eigen 矩阵（避免额外依赖 tf2_eigen）
      tf2::Quaternion q;
      tf2::fromMsg(tf->transform.rotation, q);
      Eigen::Affine3f affine = Eigen::Affine3f::Identity();
      affine.linear() = Eigen::Quaternionf(static_cast<float>(q.w()), static_cast<float>(q.x()),
                                           static_cast<float>(q.y()),
                                           static_cast<float>(q.z())).toRotationMatrix();
      affine.translation() = Eigen::Vector3f(static_cast<float>(tf->transform.translation.x),
                                             static_cast<float>(tf->transform.translation.y),
                                             static_cast<float>(tf->transform.translation.z));
      pcl::transformPointCloud(cloud, transformed, affine.matrix());
    }

    // 射线原点必须与云同时间戳
    auto sensor_tf = lookup(planning_frame_, sensor_frame_, stamp, true);
    if (!sensor_tf) {
      return;
    }
    nav_msgs::msg::Odometry sensor_pose;
    sensor_pose.header = msg->header;
    sensor_pose.header.frame_id = planning_frame_;
    sensor_pose.child_frame_id = sensor_frame_;
    sensor_pose.pose.pose.position.x = sensor_tf->transform.translation.x;
    sensor_pose.pose.pose.position.y = sensor_tf->transform.translation.y;
    sensor_pose.pose.pose.position.z = sensor_tf->transform.translation.z;
    sensor_pose.pose.pose.orientation = sensor_tf->transform.rotation;
    sensor_pose_pub_->publish(sensor_pose);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(transformed, out);
    out.header = msg->header;
    out.header.frame_id = planning_frame_;

    channel.last_valid = recv;
    channel.last_receive = recv;
    channel.last_frame = msg->header.frame_id;
    channel.error.clear();
    cloud_pub_->publish(out);
  }

  // ------------------------------------------------------------------ 健康
  void publish_health() {
    const double now = now_s();
    std::vector<std::string> reasons = collect_reasons(now);
    const bool healthy = reasons.empty();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sentry_scan/inputs";
    status.hardware_id = "rm_cpp_adapter";
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = healthy ? "OK" : reasons.front();
    for (const auto &[name, channel] : channels_) {
      auto add = [&status](const std::string &key, const std::string &value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
      };
      add(name + ".count", std::to_string(channel.count));
      add(name + ".rejected", std::to_string(channel.rejected));
      add(name + ".frame", channel.last_frame);
      add(name + ".age_s", channel.last_valid > 0.0
                                ? std::to_string(now - channel.last_valid)
                                : std::string("n/a"));
    }
    diagnostic_msgs::msg::KeyValue velocity_kv;
    velocity_kv.key = "odom.velocity";
    velocity_kv.value = velocity_reason_;
    status.values.push_back(velocity_kv);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    array.status.push_back(status);
    health_pub_->publish(array);

    std_msgs::msg::Bool latch;
    latch.data = healthy;
    health_ok_pub_->publish(latch);

    if (healthy) {
      if (!health_ok_ && ever_healthy_) {
        RCLCPP_WARN(get_logger(),
                    "Inputs healthy again; the previous task is NOT resumed.");
      }
      health_ok_ = true;
      ever_healthy_ = true;
      return;
    }
    health_ok_ = false;
    if (reset_on_unhealthy_ && ever_healthy_ &&
        (now - last_reset_) >= reset_repeat_period_) {
      last_reset_ = now;
      std_msgs::msg::Bool reset;
      reset.data = true;
      reset_pub_->publish(reset);
      RCLCPP_ERROR(get_logger(), "Inputs unhealthy -> publishing planning/reset: %s",
                   status.message.c_str());
    }
  }

  std::vector<std::string> collect_reasons(double now) {
    std::vector<std::string> reasons;
    if (jump_latched_ && now < jump_latched_until_) {
      reasons.push_back(jump_reason_);
    }
    for (const auto &[name, channel] : channels_) {
      if (!channel.error.empty()) {
        reasons.push_back(name + ": " + channel.error);
      }
      const bool fresh = channel.last_valid > 0.0 && (now - channel.last_valid) <= max_receive_age_;
      if (!fresh) {
        reasons.push_back(name + ": no valid sample within " +
                          std::to_string(max_receive_age_) + "s (count=" +
                          std::to_string(channel.count) + " rejected=" +
                          std::to_string(channel.rejected) + ")");
      }
    }
    return reasons;
  }

  // ------------------------------------------------------------------ 参数/状态
  std::string planning_frame_, body_frame_, sensor_frame_;
  std::string odom_topic_, velocity_topic_, cloud_topic_;
  std::string velocity_frame_, velocity_frame_alias_;
  std::vector<double> body_center_offset_, imu_to_body_offset_;
  double max_source_age_{}, max_receive_age_{}, max_stamp_regression_{}, max_future_stamp_{};
  int min_valid_points_{};
  double tf_lookup_timeout_{}, tf_future_tolerance_{};
  bool odom_in_planning_frame_{}, require_center_velocity_{}, require_angular_velocity_{};
  bool cloud_assume_planning_frame_{};
  double health_period_{}, reset_repeat_period_{};
  bool reset_on_unhealthy_{};
  double jump_translation_{}, jump_rotation_{}, jump_latch_duration_{};
  bool jump_latched_{false};
  double jump_latched_until_{0.0};
  std::string jump_reason_;
  double last_reset_{0.0};
  bool health_ok_{false};
  bool ever_healthy_{false};

  std::map<std::string, Channel> channels_;
  nav_msgs::msg::Odometry velocity_msg_;
  bool velocity_valid_{false};
  double velocity_stamp_{0.0};
  std::string velocity_reason_{"velocity_missing"};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pose_pub_, sensor_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_ok_pub_, reset_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sentry_scan_adapter_cpp

#ifndef RM_INPUT_ADAPTER_UNIT_TEST
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_scan_adapter_cpp::RmInputAdapter>());
  rclcpp::shutdown();
  return 0;
}
#endif
