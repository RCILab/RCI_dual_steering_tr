#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "../include/SteerDrive2WKinematics.hpp"

#define _USE_MATH_DEFINES

inline double fold_half_pi_and_flip(double theta, double& speed) {
    // r: theta를 π로 접은 나머지 ([-π/2, +π/2]에 옴)
    // k: 접힐 때 사용된 π의 "정수 배수" 정보(부호/하위 비트)
    int k = 0;
    double r = std::remquo(theta, M_PI, &k); // C++ 표준: remainder와 동일한 나머지 + 몫 비트 제공

    // π의 홀수배만큼 접혔다면 구동 방향을 반전
    if (k & 1) speed = -speed;

    const double HALF_PI = 0.5 * M_PI;

    // 반개구간 보장: [+π/2]가 나오면 [-π/2]로 보내고 속도 부호 한 번 더 반전
    if (r >= HALF_PI) {
        r -= M_PI;       // +π/2 → -π/2
        speed = -speed;  // π를 한 번 더 접었으므로 부호 보정
    }

    // 수치적 안전장치(희박하지만 부동소수 오차로 -π/2를 살짝 벗어나면 되돌림)
    if (r < -HALF_PI) {
        r += M_PI;
        speed = -speed;
    }
    return r; // 항상 [-π/2, π/2)
}

void SteerDrive2WKinematics::execForwKin(const std::shared_ptr<const sensor_msgs::msg::JointState>& js,
                                         nav_msgs::msg::Odometry& odom)
{
    auto index_of = [&](const std::string& name)->int {
      for (size_t i = 0; i < js->name.size(); ++i)
        if (js->name[i] == name) return static_cast<int>(i);
      return -1;
    };
    const int i_fd = index_of(drive_joints[0]); // front drive
    const int i_rd = index_of(drive_joints[1]); // rear  drive
    const int i_fs = index_of(steer_joints[0]); // front steer
    const int i_rs = index_of(steer_joints[1]); // rear  steer

    if (i_fd < 0 || i_rd < 0 || i_fs < 0 || i_rs < 0) {
      // 이름 매칭 실패: 업데이트 중단
      return;
    }

    const double v_f = js->velocity[i_fd] * (diam * 0.5);
    const double v_r = js->velocity[i_rd] * (diam * 0.5);
    const double th_f = js->position[i_fs];
    const double th_r = js->position[i_rs];

    const double v_cx = 0.5 * (v_f * std::cos(th_f) + v_r * std::cos(th_r));
    const double v_cy = 0.5 * (v_f * std::sin(th_f) + v_r * std::sin(th_r));
    const double v_c = std::hypot(v_cx, v_cy);
    const double beta = std::atan2(v_cy, v_cx);

    double w = 0.0;
    const double DEN_EPS  = 1e-4;
    const double NUM_EPS  = 1e-4;

    const double numer = std::sin(M_PI/2.0 - th_f + alpha) * l_f;
    const double denom = std::sin(th_f - beta);
    
    if ((std::fabs(denom) < DEN_EPS) && (v_f * v_r > 0.0)) {
      w = 0.0;
    }
    else if ((std::fabs(numer) < NUM_EPS) && (v_f * v_r < 0.0)){
      w = -v_f / l_f;
    }
    else {
      const double R = numer / denom;
      w = v_c / R;
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
  pt.time_from_start = rclcpp::Duration::from_seconds(0.02); // 20 ms (예시)

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