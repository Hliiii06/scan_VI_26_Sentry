// 全向哨兵闭环跟踪器。
//
// 与上游 SCAN-Planner ROS 2（main 103bce4）的差异：
//   1. 取消“先对齐轨迹切线朝向才允许平移”的门槛。上游在 |yaw_error| >
//      heading_error_threshold 时只发原地转向并冻结轨迹时间；RM 侧的 UART 不
//      转发 angular.z，全向底盘会永远停在冻结态。这里改为始终按实际 odom 姿态
//      把世界系速度旋转到机体系，朝向与平移互不阻塞。
//   2. 新增 yaw_mode，把“机身朝向”与“运动方向”解耦：
//        hold  —— 保持接收轨迹时的机身朝向，横移时车头不变（哨兵默认）；
//        align —— 跟踪轨迹切线朝向，但不再阻塞平移；
//        spin  —— 叠加恒定自转速率 spin_rate，用于“边转边走”包络测试。
//   3. 收到新轨迹时用消息自带的 start_time 对齐执行时间，而不是无条件把
//      exec_time 清零，使其与 FSM 的 local_data_.start_time_ 时序一致。
//   4. 增加 odom 有效性/新鲜度检查、显式 reset（取消）输入与轨迹合法性校验。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/task_authorization.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    max_vy_ = declare_parameter<double>("max_vy", 1.0);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 1.5);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    yaw_mode_ = declare_parameter<std::string>("yaw_mode", "hold");
    spin_rate_ = declare_parameter<double>("spin_rate", 0.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    start_time_align_limit_ = declare_parameter<double>("start_time_align_limit", 0.5);
    future_time_tolerance_ = declare_parameter<double>("future_time_tolerance", 0.05);
    allow_missing_start_time_ = declare_parameter<bool>("allow_missing_start_time", false);
    require_task_authorization_ = declare_parameter<bool>("require_task_authorization", true);
    // 影子接入：不透传 yaw 候选（MCU 保留朝向所有权）。默认 true 保持仿真行为不变，
    // 也不把 wz 偷塞进 linear.z —— 只是把 angular.z 置零。
    yaw_candidate_enabled_ = declare_parameter<bool>("yaw_candidate_enabled", true);

    if (yaw_mode_ != "hold" && yaw_mode_ != "align" && yaw_mode_ != "spin")
      throw std::runtime_error("yaw_mode must be 'hold', 'align' or 'spin'");
    if (odom_timeout_ <= 0.0)
      throw std::runtime_error("odom_timeout must be positive");
    if (start_time_align_limit_ <= 0.0)
      throw std::runtime_error("start_time_align_limit must be positive");

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "planning/reset", 10,
        std::bind(&ClosedLoopController::resetCallback, this, std::placeholders::_1));
    // 任务授权：取消后必须由规划端显式重新授权才允许执行。
    // 只靠时间戳推断"这条轨迹属于新任务"是不完整的（例如无 start_time 的轨迹）。
    task_active_sub_ = create_subscription<scan_planner_msgs::msg::TaskAuthorization>(
        "planning/task_active", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&ClosedLoopController::taskActiveCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(),
                "Omnidirectional closed-loop tracker ready: yaw_mode=%s, "
                "vx=%.2f, vy=%.2f, wz=%.2f, odom_timeout=%.2fs, "
                "trajectory age limit=%.2fs (older/future trajectories are rejected), "
                "require_task_authorization=%s, allow_missing_start_time=%s, "
                "yaw_candidate_enabled=%s",
                yaw_mode_.c_str(), max_vx_, max_vy_, max_vyaw_, odom_timeout_,
                start_time_align_limit_, require_task_authorization_ ? "true" : "false",
                allow_missing_start_time_ ? "true" : "false",
                yaw_candidate_enabled_ ? "true" : "false");
  }

private:
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = yaw_candidate_enabled_ ? std::clamp(yaw_rate, -max_vyaw_, max_vyaw_) : 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void clearTrajectory(const std::string &reason)
  {
    if (receive_traj_)
      RCLCPP_WARN(get_logger(), "Trajectory cleared: %s", reason.c_str());
    receive_traj_ = false;
    traj_.clear();
    traj_duration_ = 0.0;
    exec_time_ = 0.0;
  }

  void taskActiveCallback(const scan_planner_msgs::msg::TaskAuthorization::ConstSharedPtr msg)
  {
    if (msg->task_id > last_task_id_)
      last_task_id_ = msg->task_id;

    if (!msg->active)
    {
      // 规划端撤权：本地锁止到该编号，之后编号不大于它的授权一律不放行。
      if (msg->task_id > revoked_up_to_)
        revoked_up_to_ = msg->task_id;
      if (task_active_)
      {
        task_active_ = false;
        clearTrajectory("task authorization revoked by planner");
        publishExecutionFrozen(false);
        publishStop();
      }
      RCLCPP_WARN(get_logger(), "Task authorization REVOKED by planner (task_id=%u)",
                  msg->task_id);
      return;
    }

    // 只接受**编号大于已撤销编号**的授权。取消之前发布、延迟到达的旧授权
    // 编号不会更大，因此不会把已锁止的执行端重新放行。
    if (msg->task_id <= revoked_up_to_)
    {
      RCLCPP_WARN(get_logger(),
                  "Ignoring stale task authorization task_id=%u (revoked up to %u)",
                  msg->task_id, revoked_up_to_);
      return;
    }

    if (task_active_)
      return;
    task_active_ = true;
    RCLCPP_WARN(get_logger(), "Task authorization GRANTED (task_id=%u)", msg->task_id);
  }

  /// 收到取消时**立即在本地锁止**，不等规划端发布撤权。
  /// 否则规划端还在计算的旧任务会发布一条 start_time 晚于取消时刻的轨迹，
  /// 在本端看来是"新"的并被接受——这正是此前遗留的竞态窗口。
  void lockOutLocally(const char *reason)
  {
    if (last_task_id_ > revoked_up_to_)
      revoked_up_to_ = last_task_id_;
    task_active_ = false;
    RCLCPP_WARN(get_logger(), "Local execution lockout (%s); revoked up to task_id=%u",
                reason, revoked_up_to_);
  }

  void resetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (!msg->data)
      return;
    // 记录取消时刻：此后到达的、start_time 早于该时刻的轨迹一律拒绝，
    // 防止取消前发出、取消后才送达的在途轨迹重新驱动机器人。
    cancel_time_ = now();
    // 先本地锁止执行授权，再清轨迹：规划端若仍在计算旧任务，其随后发布的
    // 轨迹与授权都不能再让本端动起来（Codex 第二轮指出的竞态窗口）。
    lockOutLocally("cancel received");
    clearTrajectory("reset requested; stopping and forgetting the active trajectory");
    publishExecutionFrozen(false);
    publishStop();
    RCLCPP_WARN(get_logger(),
                "Cancel latched at %.3f; trajectories older than this will be rejected",
                cancel_time_.seconds());
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline (order/pts/knots)");
      return;
    }
    if (msg->pos_pts.size() < static_cast<size_t>(msg->order) + 1 ||
        msg->knots.size() != msg->pos_pts.size() + msg->order + 1)
    {
      RCLCPP_WARN(get_logger(), "Ignoring malformed B-spline: %zu points, %zu knots, order %d",
                  msg->pos_pts.size(), msg->knots.size(), msg->order);
      return;
    }

    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      const auto &pt = msg->pos_pts[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        RCLCPP_WARN(get_logger(), "Ignoring B-spline with non-finite control point");
        return;
      }
      points.col(i) << pt.x, pt.y, pt.z;
    }
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
      if (!std::isfinite(msg->knots[i]))
      {
        RCLCPP_WARN(get_logger(), "Ignoring B-spline with non-finite knot");
        return;
      }
      knots(i) = msg->knots[i];
    }

    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    const double duration = position.getTimeSum();
    if (!std::isfinite(duration) || duration <= 1e-3)
    {
      RCLCPP_WARN(get_logger(), "Ignoring B-spline with invalid duration %.4f", duration);
      return;
    }

    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = duration;
    traj_id_ = msg->traj_id;
    last_update_time_ = now();

    // ---- 轨迹时间契约 ----
    // 与 FSM 的 local_data_.start_time_ 对齐：轨迹从 start_time 开始执行。三条规则：
    //   1) start_time 早于最近一次取消 -> 拒绝（取消前发出、取消后才到的在途轨迹）。
    //   2) 已经过期（elapsed 超过容忍上限）-> 拒绝，不再从头执行，避免旧轨迹重新驱动底盘。
    //   3) start_time 明显在未来（时钟异常）-> 拒绝。
    // 只有落在容忍窗口内的轨迹才被接受，并按 elapsed 起算。
    const rclcpp::Time start_time(msg->start_time);
    if (start_time.seconds() <= 1e-5)
    {
      // 无 start_time 就无法判断新旧。取消之后必须拒绝（否则绕过取消闩锁）；
      // 平时也默认拒绝，只有显式 allow_missing_start_time 才接受。
      if (cancel_time_.seconds() > 1e-5)
      {
        RCLCPP_WARN(get_logger(),
                    "Rejecting trajectory %ld without start_time after a cancel: "
                    "cannot prove it belongs to a new task",
                    static_cast<long long>(traj_id_));
        clearTrajectory("trajectory without start_time received after a cancel");
        return;
      }
      if (!allow_missing_start_time_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Rejecting trajectory without start_time "
                             "(allow_missing_start_time is false)");
        return;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Trajectory has no start_time; accepted at t=0 because "
                           "allow_missing_start_time is true");
      exec_time_ = 0.0;
    }
    else
    {
      if (cancel_time_.seconds() > 1e-5 && start_time <= cancel_time_)
      {
        RCLCPP_WARN(get_logger(),
                    "Rejecting trajectory %ld published before the last cancel (start_time %.3f <= cancel %.3f)",
                    static_cast<long long>(traj_id_), start_time.seconds(), cancel_time_.seconds());
        clearTrajectory("stale trajectory from before the last cancel");
        return;
      }
      const double elapsed = (now() - start_time).seconds();
      if (elapsed > start_time_align_limit_)
      {
        RCLCPP_WARN(get_logger(),
                    "Rejecting expired trajectory %ld: start_time is %.3fs old (limit %.3fs)",
                    static_cast<long long>(traj_id_), elapsed, start_time_align_limit_);
        clearTrajectory("expired trajectory");
        return;
      }
      if (elapsed < -future_time_tolerance_)
      {
        RCLCPP_WARN(get_logger(),
                    "Rejecting trajectory %ld with start_time %.3fs in the future (tolerance %.3fs)",
                    static_cast<long long>(traj_id_), -elapsed, future_time_tolerance_);
        clearTrajectory("trajectory start_time is in the future");
        return;
      }
      exec_time_ = std::clamp(elapsed, 0.0, traj_duration_);
    }

    // hold 模式锁定接收轨迹瞬间的机身朝向。
    hold_yaw_ = odom_yaw_;
    have_hold_yaw_ = true;
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %ld, duration %.3fs, start at t=%.3fs",
                static_cast<long long>(traj_id_), traj_duration_, exec_time_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto &pose = msg->pose.pose;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Rejecting odometry with non-finite position");
      return;
    }
    const auto &q = pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Rejecting odometry with invalid orientation quaternion");
      return;
    }
    tf2::Quaternion quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
    odom_pos_ << pose.position.x, pose.position.y, pose.position.z;
    odom_yaw_ = tf2::getYaw(quaternion);
    last_odom_time_ = now();
    have_odom_ = true;
  }

  void cmdCallback()
  {
    const auto current_time = now();

    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }

    if (require_task_authorization_ && !task_active_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Task is not authorized; holding position");
      publishExecutionFrozen(false);
      publishStop();
      return;
    }

    const double odom_age = (current_time - last_odom_time_).seconds();
    if (odom_age < 0.0 || odom_age > odom_timeout_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Odometry stale (%.3fs > %.3fs); stopping", odom_age, odom_timeout_);
      // 输入失效时立即停止并遗忘轨迹，恢复后不会重新激活旧命令。
      clearTrajectory("odometry input stale");
      publishExecutionFrozen(false);
      publishStop();
      return;
    }

    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    last_update_time_ = current_time;

    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);

    // ---- 朝向：与平移解耦，任何 yaw 误差都不阻塞 XY 运动 ----
    double desired_yaw = odom_yaw_;
    if (yaw_mode_ == "align")
      desired_yaw = estimateDesiredYaw(t_eval, pos_des);
    else if (yaw_mode_ == "hold" && have_hold_yaw_)
      desired_yaw = hold_yaw_;

    double yaw_command = 0.0;
    if (yaw_mode_ == "spin")
      yaw_command = std::clamp(spin_rate_, -max_vyaw_, max_vyaw_);
    else
      yaw_command = std::clamp(kp_yaw_ * normalizeAngle(desired_yaw - odom_yaw_), -max_vyaw_, max_vyaw_);

    // ---- 平移：世界系前馈 + 比例反馈，再按实际机身 yaw 旋转到机体系 ----
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;

    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();

    if (!yaw_candidate_enabled_)
      command.angular.z = 0.0;

    publishExecutionFrozen(false);
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::TaskAuthorization>::SharedPtr task_active_sub_;
  uint32_t last_task_id_{0};    // 见过的最大任务编号
  uint32_t revoked_up_to_{0};   // 已撤销到的编号；只有更大编号的授权才放行
  bool task_active_{false};
  bool allow_missing_start_time_{false};
  bool require_task_authorization_{true};
  bool yaw_candidate_enabled_{true};
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool have_hold_yaw_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double hold_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cancel_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double spin_rate_{0.0}, odom_timeout_{0.5}, start_time_align_limit_{0.5};
  double future_time_tolerance_{0.05};
  std::string yaw_mode_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
