#pragma once
#include <vector>
#include <string>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class SteerDrive2WKinematics {
public:
  void execForwKin(const std::shared_ptr<const sensor_msgs::msg::JointState>& js,
                   nav_msgs::msg::Odometry& odom);

  void execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& twist,
                  const std::shared_ptr<const sensor_msgs::msg::JointState>& joint_state,
                  trajectory_msgs::msg::JointTrajectory& steer_traj,
                  std_msgs::msg::Float64MultiArray& drive_cmd);

  void setModuleSpec(double wheelDiameter,
                     std::vector<std::string> steerJoints, std::vector<std::string> driveJoints,
                     std::vector<double> moduleX, std::vector<double> moduleY);

  const std::vector<std::string>& steerJointNames() const { return steer_joints; }
  const std::vector<std::string>& driveJointNames() const { return drive_joints; }

  enum class SteerMode {
    DRIVE,      // 정상 주행
    WAIT_STOP,  // 멈추는 중 (구동=0, 조향 고정)
    STEER_ALIGN // 정지 상태에서 조향만 맞추는 중
  };
private:
  nav_msgs::msg::Odometry odom_cur;

  double diam{0.0};
  double x_f{0.0}, y_f{0.0}, x_r{0.0}, y_r{0.0}, l_f{0.0}, alpha{0.0};
  double phi{0.0};

  double cur_v_f_{0.0};
  double cur_v_r_{0.0};
  double cur_th_f_{0.0};
  double cur_th_r_{0.0};
  bool   have_drive_state_{false};
  bool   have_steer_state_{false};

  double ANGLE_ERR_THRESH_{M_PI / 180.0}; // 1 deg, rad 단위
  double max_steer_rate_{1.0};             // rad/s, 조향 최대 각속도 (원하는 값으로 튜닝)
  double control_period_{0.05}; 

// [필수] 이전 속도 기억 (Cubic 계산의 시작 속도로 사용)
  double last_steer_vel_f_ = 0.0;
  double last_steer_vel_r_ = 0.0;

  double cur_v_f_filt_ = 0.0;
  double cur_v_r_filt_ = 0.0;
  int stop_confirm_counter_ = 0;
  // [튜닝] 목표까지 도달하는 데 걸리는 시간 (클수록 부드럽고 느리게 반응)
  // 0.5초 정도가 적당하며, 너무 작으면 튑니다.
  double smooth_time_ = 0.3;
  const double MAX_STEER_VEL   = 0.5;  // 최대 조향 속도 (rad/s)

  SteerMode mode_{SteerMode::DRIVE};
  std::vector<std::string> steer_joints; // size 2
  std::vector<std::string> drive_joints; // size 2
};
