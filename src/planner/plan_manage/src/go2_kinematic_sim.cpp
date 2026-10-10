#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "plan_manage/ground_height_map.h"

namespace scan_planner
{
class Go2KinematicSim : public rclcpp::Node
{
public:
  Go2KinematicSim() : Node("go2_kinematic_sim")
  {
    x_ = declare_parameter<double>("init_x", 0.0);
    y_ = declare_parameter<double>("init_y", 0.0);
    z_ = declare_parameter<double>("init_z", 0.3);
    yaw_ = declare_parameter<double>("init_yaw", 0.0);
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    max_vy_ = declare_parameter<double>("max_vy", 1.0);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.5), kMaxVYawLimit);
    max_acc_xy_ = declare_parameter<double>("max_acc_xy", 0.0);
    max_acc_yaw_ = declare_parameter<double>("max_acc_yaw", 0.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    const double sim_rate = declare_parameter<double>("sim_rate", 100.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");
    // 地形跟随：给出地面高度网格后，z 由「地面 + 机体中心高度」决定，而不是固定值。
    // 地面机器人不控制 z（z 是地形的结果），所以这里由地形推导，而不是抄轨迹的 z。
    const std::string ground_file = declare_parameter<std::string>("ground_grid_file", "");
    body_height_ = declare_parameter<double>("body_height", 0.125);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.26);
    if (!ground_file.empty())
    {
      if (!ground_map_.load(ground_file))
        throw std::runtime_error("ground_grid_file could not be loaded: " + ground_file);
      terrain_following_ = true;
      z_ = ground_map_.heightAt(x_, y_) + body_height_;
      last_valid_x_ = x_;
      last_valid_y_ = y_;
      RCLCPP_INFO(get_logger(),
                  "Terrain following ENABLED from %s: ground z in [%.3f, %.3f] m over a "
                  "%.2f m grid; body height %.3f m",
                  ground_file.c_str(), ground_map_.minHeight(), ground_map_.maxHeight(),
                  ground_map_.cell(), body_height_);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 100);
    // 实体机体：RViz 里看"车体贴着地形升降"比看圆柱包络直观。
    // 尺寸来自机器人参数（半径/高度），与碰撞包络同源。
    body_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "body_marker", rclcpp::QoS(1).reliable().transient_local());
    body_marker_.header.frame_id = "world";
    body_marker_.ns = "robot_body";
    body_marker_.id = 0;
    // 必须与碰撞包络**几何一致**：包络是半径 robot_radius 的竖直圆柱，
    // 所以显示也用同样半径的圆柱。用边长 2r 的方块是错的——方块角点到中心
    // 距离为 r*sqrt(2)（0.26 -> 0.368 m），四角会露在包络之外，
    // 出现"画面里车角碰墙、规划器却认为安全"的假象。
    body_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
    body_marker_.action = visualization_msgs::msg::Marker::ADD;
    // CYLINDER 的 scale.x/y 是直径，scale.z 是全高；位置在机体中心
    body_marker_.scale.x = 2.0 * robot_radius_;
    body_marker_.scale.y = 2.0 * robot_radius_;
    body_marker_.scale.z = 2.0 * body_height_;
    body_marker_.color.r = 0.25f;
    body_marker_.color.g = 0.65f;
    body_marker_.color.b = 1.0f;
    body_marker_.color.a = 0.85f;
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 20, std::bind(&Go2KinematicSim::cmdCallback, this, std::placeholders::_1));
    last_cmd_time_ = now();
    last_sim_time_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, sim_rate)),
        std::bind(&Go2KinematicSim::simCallback, this));
    RCLCPP_INFO(get_logger(),
                "Holonomic kinematic simulator ready: init=(%.2f, %.2f, %.2f) yaw=%.2f, "
                "limits vx=%.2f vy=%.2f wz=%.2f, acc xy=%.2f yaw=%.2f, cmd_timeout=%.2fs, "
                "terrain_following=%s",
                x_, y_, z_, yaw_, max_vx_, max_vy_, max_vyaw_, max_acc_xy_, max_acc_yaw_,
                cmd_timeout_, terrain_following_ ? "true" : "false");
  }

private:
  static constexpr double kMaxVYawLimit = 1.5;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_cmd_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    vy_cmd_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    vyaw_cmd_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    last_cmd_time_ = now();
  }

  void publishBodyMarker(const rclcpp::Time &stamp)
  {
    body_marker_.header.stamp = stamp;
    body_marker_.pose.position.x = x_;
    body_marker_.pose.position.y = y_;
    body_marker_.pose.position.z = z_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    body_marker_.pose.orientation = tf2::toMsg(q);
    body_marker_pub_->publish(body_marker_);
  }

  void publishOdom(const rclcpp::Time &stamp)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    const auto orientation = tf2::toMsg(quaternion);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = vx_world_;
    odom.twist.twist.linear.y = vy_world_;
    odom.twist.twist.angular.z = wz_applied_;
    odom_pub_->publish(odom);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = z_;
      transform.transform.rotation = orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void simCallback()
  {
    const auto current_time = now();
    double dt = (current_time - last_sim_time_).seconds();
    last_sim_time_ = current_time;
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    double vx = vx_cmd_, vy = vy_cmd_, wz = vyaw_cmd_;
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_)
      vx = vy = wz = 0.0;

    // 可选一阶加速度限制：默认 0（关闭）保持上游行为；哨兵仿真打开后速度不能
    // 瞬间跳变，闭环跟踪误差才有意义（不能无条件精确贴合样条）。
    if (dt > 0.0 && max_acc_xy_ > 0.0)
    {
      const double dvx = vx - vx_applied_;
      const double dvy = vy - vy_applied_;
      const double norm = std::hypot(dvx, dvy);
      const double max_delta = max_acc_xy_ * dt;
      if (norm > max_delta && norm > 1e-9)
      {
        vx_applied_ += dvx / norm * max_delta;
        vy_applied_ += dvy / norm * max_delta;
      }
      else
      {
        vx_applied_ = vx;
        vy_applied_ = vy;
      }
    }
    else
    {
      vx_applied_ = vx;
      vy_applied_ = vy;
    }

    if (dt > 0.0 && max_acc_yaw_ > 0.0)
    {
      const double max_delta = max_acc_yaw_ * dt;
      wz_applied_ += std::clamp(wz - wz_applied_, -max_delta, max_delta);
    }
    else
    {
      wz_applied_ = wz;
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    vx_world_ = c * vx_applied_ - s * vy_applied_;
    vy_world_ = s * vx_applied_ + c * vy_applied_;
    const double next_x = x_ + vx_world_ * dt;
    const double next_y = y_ + vy_world_ * dt;
    if (terrain_following_ && !ground_map_.contains(next_x, next_y))
    {
      // **停在边界上，不进入未知区域。** 只保持高度继续走仍然是把未知区域当成了
      // 可行驶地面；这里直接把位置钉在最后一个已知点并清零速度。
      ++out_of_grid_events_;
      x_ = last_valid_x_;
      y_ = last_valid_y_;
      vx_applied_ = 0.0;
      vy_applied_ = 0.0;
      vx_world_ = 0.0;
      vy_world_ = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Robot reached the edge of the known ground grid "
                           "(x[%.2f,%.2f] y[%.2f,%.2f]) at (%.2f, %.2f); holding position "
                           "instead of driving into unknown ground (%d events)",
                           ground_map_.minX(), ground_map_.maxX(),
                           ground_map_.minY(), ground_map_.maxY(),
                           x_, y_, out_of_grid_events_);
    }
    else
    {
      x_ = next_x;
      y_ = next_y;
    }
    if (terrain_following_)
    {
      z_ = ground_map_.heightAt(x_, y_) + body_height_;
      last_valid_x_ = x_;
      last_valid_y_ = y_;
      last_valid_x_ = x_;
      last_valid_y_ = y_;
    }
    yaw_ = normalizeAngle(yaw_ + wz_applied_ * dt);
    publishOdom(current_time);
    publishBodyMarker(current_time);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_{0.0}, y_{0.0}, z_{0.3}, yaw_{0.0};
  double vx_cmd_{0.0}, vy_cmd_{0.0}, vyaw_cmd_{0.0};
  double vx_world_{0.0}, vy_world_{0.0};
  double max_vx_{1.0}, max_vy_{1.0}, max_vyaw_{1.5}, cmd_timeout_{0.3};
  double max_acc_xy_{0.0}, max_acc_yaw_{0.0};
  double vx_applied_{0.0}, vy_applied_{0.0}, wz_applied_{0.0};
  bool publish_tf_{false};
  bool terrain_following_{false};
  double body_height_{0.125};
  double robot_radius_{0.26};
  double last_valid_x_{0.0}, last_valid_y_{0.0};
  long out_of_grid_events_{0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_marker_pub_;
  visualization_msgs::msg::Marker body_marker_;
  GroundHeightMap ground_map_;
  std::string frame_id_, child_frame_id_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2KinematicSim>());
  rclcpp::shutdown();
  return 0;
}
