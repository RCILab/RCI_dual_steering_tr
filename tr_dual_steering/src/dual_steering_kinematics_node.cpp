#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "tr_dual_steering/dual_steering_types.hpp"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

namespace tr_dual_steering {

class DualSteeringKinematicsNode : public rclcpp::Node {
public:
  DualSteeringKinematicsNode() : Node("dual_steering_kinematics_node") {
    // 기본 파라미터
    declare_parameter<double>("wheel_radius", params_.wheel_radius);
    declare_parameter<double>("publish_rate_hz", params_.publish_rate_hz);
    declare_parameter<std::string>("cmd_vel_topic", params_.cmd_vel_topic);
    declare_parameter<std::string>("traj_topic", params_.traj_topic);

    // 모듈(바퀴) 정의 — 배열로 일반화
    // 예시 기본값: 2모듈(전/후), L=0.87 -> x=[+0.435,-0.435], y=[0,0]
    declare_parameter<std::vector<std::string>>("module_names", {"front","rear"});
    declare_parameter<std::vector<std::string>>("steer_joints", {"front_steer_joint","rear_steer_joint"});
    declare_parameter<std::vector<std::string>>("drive_joints", {"front_drive_joint","rear_drive_joint"});
    declare_parameter<std::vector<double>>("module_x", {+0.435, -0.435});
    declare_parameter<std::vector<double>>("module_y", {0.0, 0.0});

    // 로드
    get_parameter("wheel_radius", params_.wheel_radius);
    get_parameter("publish_rate_hz", params_.publish_rate_hz);
    get_parameter("cmd_vel_topic", params_.cmd_vel_topic);
    get_parameter("traj_topic", params_.traj_topic);

    std::vector<std::string> names, steer_js, drive_js;
    std::vector<double> xs, ys;
    get_parameter("module_names", names);
    get_parameter("steer_joints", steer_js);
    get_parameter("drive_joints", drive_js);
    get_parameter("module_x", xs);
    get_parameter("module_y", ys);

    build_modules_from_arrays(names, steer_js, drive_js, xs, ys);

    // I/O
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        params_.cmd_vel_topic, rclcpp::QoS(10),
        std::bind(&DualSteeringKinematicsNode::on_cmd_vel, this, _1));

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        params_.traj_topic, rclcpp::QoS(10));

    // 타이머
    double rate = std::max(1.0, params_.publish_rate_hz);  // 둘 다 double
    const double dt = 1.0 / rate;
    timer_ = create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&DualSteeringKinematicsNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "tr_dual_steering generic node started. modules=%zu, wheel_radius=%.3f",
      modules_.size(), params_.wheel_radius);
  }

private:
  void build_modules_from_arrays(const std::vector<std::string>& names,
                                 const std::vector<std::string>& steer_js,
                                 const std::vector<std::string>& drive_js,
                                 const std::vector<double>& xs,
                                 const std::vector<double>& ys) {
    const size_t n = std::min({names.size(), steer_js.size(), drive_js.size(), xs.size(), ys.size()});
    if (n < 1) {
      RCLCPP_FATAL(get_logger(), "No modules configured. Check module_* parameters.");
      throw std::runtime_error("No modules configured");
    }
    if (n < names.size() || n < steer_js.size() || n < drive_js.size() || n < xs.size() || n < ys.size()) {
      RCLCPP_WARN(get_logger(), "Parameter array sizes differ; using first %zu entries.", n);
    }
    modules_.clear();
    modules_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      ModuleSpec m;
      m.name = names[i];
      m.steer_joint = steer_js[i];
      m.drive_joint = drive_js[i];
      m.x = xs[i];
      m.y = ys[i];
      modules_.push_back(m);
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_wz_ = msg->angular.z;
  }

  // 강체 속도장 기반 모듈 kinematics
  inline void compute_module_kinematics(double x, double y,
                                        double vx, double vy, double wz,
                                        double &steer_angle, double &wheel_speed_mps) {
    const double vix = vx - wz * y;
    const double viy = vy + wz * x;
    steer_angle = std::atan2(viy, vix);        // [rad]
    wheel_speed_mps = std::hypot(vix, viy);    // [m/s]
  }

  void on_timer() {
    if (modules_.empty()) return;
    const double dt = 1.0 / params_.publish_rate_hz;

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->get_clock()->now();
    traj.joint_names.reserve(modules_.size() * 2);
    for (auto &m : modules_) traj.joint_names.push_back(m.steer_joint);
    for (auto &m : modules_) traj.joint_names.push_back(m.drive_joint);

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start = rclcpp::Duration::from_seconds(dt);

    const double vx = latest_vx_;
    const double vy = latest_vy_;
    const double wz = latest_wz_;

    std::vector<double> steer_positions; steer_positions.reserve(modules_.size());
    std::vector<double> drive_positions; drive_positions.reserve(modules_.size());

    for (auto &m : modules_) {
      double theta=0.0, speed_mps=0.0;
      compute_module_kinematics(m.x, m.y, vx, vy, wz, theta, speed_mps);
      // 라핑
      theta = std::atan2(std::sin(theta), std::cos(theta));
      m.steer_angle = theta;

      // 바퀴 각속도(rad/s)로 변환 후 적분
      const double wheel_w = (params_.wheel_radius > 1e-9)
                            ? (speed_mps / params_.wheel_radius) : 0.0;
      m.drive_pos += wheel_w * dt;

      steer_positions.push_back(m.steer_angle);
      drive_positions.push_back(m.drive_pos);
    }

    pt.positions.reserve(modules_.size() * 2);
    for (double a : steer_positions) pt.positions.push_back(a);
    for (double p : drive_positions) pt.positions.push_back(p);

    traj.points.push_back(pt);
    pub_->publish(traj);
  }

private:
  Params params_;
  std::vector<ModuleSpec> modules_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double latest_vx_{0.0}, latest_vy_{0.0}, latest_wz_{0.0};
};

} // namespace tr_dual_steering

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tr_dual_steering::DualSteeringKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
