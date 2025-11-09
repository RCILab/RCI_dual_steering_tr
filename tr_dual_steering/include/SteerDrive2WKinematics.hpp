#pragma once
#include <vector>
#include <string>
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
                  trajectory_msgs::msg::JointTrajectory& steer_traj,
                  std_msgs::msg::Float64MultiArray& drive_cmd);

  void setModuleSpec(double wheelDiameter,
                     std::vector<std::string> steerJoints, std::vector<std::string> driveJoints,
                     std::vector<double> moduleX, std::vector<double> moduleY);

  const std::vector<std::string>& steerJointNames() const { return steer_joints; }
  const std::vector<std::string>& driveJointNames() const { return drive_joints; }

private:
  nav_msgs::msg::Odometry odom_cur;

  double diam{0.0};
  double x_f{0.0}, y_f{0.0}, x_r{0.0}, y_r{0.0}, l_f{0.0}, alpha{0.0};
  double phi{0.0};

  std::vector<std::string> steer_joints; // size 2
  std::vector<std::string> drive_joints; // size 2
};
