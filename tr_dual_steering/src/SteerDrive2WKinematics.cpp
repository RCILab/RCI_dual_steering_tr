#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "../include/SteerDrive2WKinematics.hpp"

#define _USE_MATH_DEFINES

inline double fold_half_pi_and_flip(double theta, double& speed) {
    double wrapped = std::remainder(theta, M_PI);

    if (std::cos(theta - wrapped) < 0.0) {
        speed = -speed;
    }
    return wrapped;
}

void SteerDrive2WKinematics::execForwKin(const std::shared_ptr<const sensor_msgs::msg::JointState>& js,
                                         nav_msgs::msg::Odometry& odom)
{
    const double v_f = js->velocity[0] * (diam * 0.5);
    const double v_r = js->velocity[1] * (diam * 0.5);
    const double th_f = js->position[2];
    const double th_r = js->position[3];

    const double v_cx = v_f * std::cos(th_f) + v_r * std::cos(th_r);
    const double v_cy = v_f * std::sin(th_f) + v_r * std::sin(th_r);
    const double v_c = std::hypot(v_cx, v_cy);
    const double beta = std::atan2(v_cy, v_cx);
    
    double w = 0.0;

    const double LIN_EPS  = 1e-3;   // m/s
    const double DEN_EPS  = 1e-4;
    const double NUM_EPS  = 1e-4;
    const double R_EPS    = 1e-2;   // m, "거의 0" 반경
    const double WHEEL_EPS = 1e-3;

    const double numer = std::sin(M_PI/2.0 - th_f + alpha) * l_f;
    const double denom   = std::sin(th_f - beta);

    if (v_c > LIN_EPS && std::fabs(denom) > DEN_EPS && std::fabs(numer) > NUM_EPS) {
      const double R = numer/ denom;
      if (std::fabs(R) < R_EPS) {
        w = v_c / R;
      }
      else {
        // 속도가 너무 작거나, th_f ≈ beta 이면 특이점이므로 회전 없음으로 간주
        w = 0.0;
      }
    }
    else{
      // 2) v_c가 거의 0인데 휠은 돌고 있는 경우 -> 제자리 회전으로 본다
      if (v_c < LIN_EPS &&
          (std::fabs(v_f) > WHEEL_EPS || std::fabs(v_r) > WHEEL_EPS))
      {
          // 제자리 회전용 w 계산식 (간단 버전 예시)
          // 실제로는 조향 전략을 보고 부호/계수 맞춰야 함.
          const double l_f = std::hypot(x_f, y_f);
          const double l_r = std::hypot(x_r, y_r);

          double w_f = 0.0, w_r = 0.0;
          if (l_f > 1e-4) w_f = -v_f / l_f;
          if (l_r > 1e-4) w_r = -v_r / l_r;

          w = 0.5 * (w_f + w_r);  // 단순 평균 (필요하면 더 잘 튜닝)
      }
      else {
          w = 0.0;  // 거의 정지
      }
    }

	if(odom_cur.header.stamp.sec != 0)
	{
		const double t_js = rclcpp::Time(js->header.stamp).seconds();
		const double t_odom = rclcpp::Time(odom_cur.header.stamp).seconds();
		const double dt = t_js - t_odom;

		const double v_x_mid = 0.5 * (v_cx + odom_cur.twist.twist.linear.x);
    const double v_y_mid = 0.5 * (v_cy + odom_cur.twist.twist.linear.y);
		const double w_mid = 0.5 * (w + odom_cur.twist.twist.angular.z);
		const double phi_mid = phi + w_mid * dt * 0.5;
    
		odom_cur.pose.pose.position.x += v_x_mid * dt * cos(phi_mid) - v_y_mid * sin(phi_mid) * dt;
		odom_cur.pose.pose.position.y += v_x_mid * dt * sin(phi_mid) + v_y_mid * cos(phi_mid) * dt;
		odom_cur.pose.pose.position.z = 0;
		phi += w_mid * dt;
		tf2::Quaternion q;
		q.setRPY(0, 0, phi);
		odom_cur.pose.pose.orientation = tf2::toMsg(q);
	}
	odom_cur.header.stamp = js->header.stamp;
	odom_cur.header.frame_id = odom.header.frame_id;
	odom_cur.child_frame_id = odom.child_frame_id;
	odom_cur.twist.twist.linear.x = v_cx;
  odom_cur.twist.twist.linear.y = v_cy;
	odom_cur.twist.twist.angular.z = w;
	odom = odom_cur;
}

void SteerDrive2WKinematics::execInvKin(
  const std::shared_ptr<const geometry_msgs::msg::Twist>& twist,
  trajectory_msgs::msg::JointTrajectory& steer_traj,
  std_msgs::msg::Float64MultiArray& drive_cmd)
{
  const double v_fx = twist->linear.x - twist->angular.z * y_f;
  const double v_fy = twist->linear.y + twist->angular.z * x_f;
  const double v_rx = twist->linear.x - twist->angular.z * y_r;
  const double v_ry = twist->linear.y + twist->angular.z * x_r;

  double th_f = std::atan2(v_fy, v_fx);
  double th_r = std::atan2(v_ry, v_rx);
  double v_f = std::hypot(v_fx, v_fy) * 2.0 / diam;
  double v_r = std::hypot(v_rx, v_ry) * 2.0 / diam;

  th_f = fold_half_pi_and_flip(th_f, v_f);
  th_r = fold_half_pi_and_flip(th_r, v_r);

  steer_traj = trajectory_msgs::msg::JointTrajectory{};
  steer_traj.joint_names = steer_joints;      // size == 2
  steer_traj.points.resize(1);
  auto &pt = steer_traj.points[0];
  pt.positions = {th_f, th_r};                // pos only
  pt.time_from_start = rclcpp::Duration::from_seconds(0.5); // 20 ms (예시)

  drive_cmd = std_msgs::msg::Float64MultiArray{};
  drive_cmd.data = {v_f, v_r};
}

void SteerDrive2WKinematics::setModuleSpec(
  double wheelDiameter, std::vector<std::string> steerJoints,
  std::vector<std::string> driveJoints, std::vector<double> moduleX,
  std::vector<double> moduleY)
{
  diam = wheelDiameter;
  steer_joints = std::move(steerJoints);
  drive_joints = std::move(driveJoints);

  x_f = moduleX[0];
  y_f = moduleY[0];
  x_r = moduleX[1];
  y_r = moduleY[1];
  l_f = std::hypot(x_f, y_f);
  alpha = std::atan2(y_f, x_f);
}