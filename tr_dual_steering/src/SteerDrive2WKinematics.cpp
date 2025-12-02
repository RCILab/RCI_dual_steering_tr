#include <cmath>

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "../include/SteerDrive2WKinematics.hpp"

#define _USE_MATH_DEFINES
using SteerMode = SteerDrive2WKinematics::SteerMode;

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

    cur_v_f_ = v_f;
    cur_v_r_ = v_r;
    cur_th_f_ = th_f;
    cur_th_r_ = th_r;
    have_drive_state_ = true;
    have_steer_state_ = true;

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
		odom_cur.pose.pose.position.z = 0.0;
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
  const std::shared_ptr<const sensor_msgs::msg::JointState>& joint_state,
  trajectory_msgs::msg::JointTrajectory& steer_traj,
  std_msgs::msg::Float64MultiArray& drive_cmd
)
{
  // Get Command
  const double v_fx = twist->linear.x - twist->angular.z * y_f;
  const double v_fy = twist->linear.y + twist->angular.z * x_f;
  const double v_rx = twist->linear.x - twist->angular.z * y_r;
  const double v_ry = twist->linear.y + twist->angular.z * x_r;

  // Command Filtering
  double th_f_target;
  double th_r_target;
  double v_f_target;
  double v_r_target;

  // Target
  const double eps = 1e-6;
  if (std::hypot(v_fx, v_fy) < eps && std::hypot(v_rx, v_ry) < eps) {
    // 거의 안 움직이는 상황 → 여기서는 조향 target 정책을 직접 정할 수 있음
    th_f_target = 0.0;
    th_r_target = 0.0;
    v_f_target  = 0.0;
    v_r_target  = 0.0;
  } else {
    th_f_target = std::atan2(v_fy, v_fx);
    th_r_target = std::atan2(v_ry, v_rx);
    v_f_target  = std::hypot(v_fx, v_fy) * 2.0 / diam;
    v_r_target  = std::hypot(v_rx, v_ry) * 2.0 / diam;
  }
  // [-90,90)범위에서 (전/후진 모터 회전방향 전환)
  th_f_target = fold_half_pi_and_flip(th_f_target, v_f_target);
  th_r_target = fold_half_pi_and_flip(th_r_target, v_r_target);

  // Get Joint State
  double cur_th_f = 0.0;
  double cur_th_r = 0.0;
  double cur_v_f  = 0.0;
  double cur_v_r  = 0.0;

  const std::string JOINTS_STEER_FRONT = "front_steer_joint";
  const std::string JOINTS_STEER_REAR  = "rear_steer_joint";
  const std::string JOINTS_DRIVE_FRONT = "front_drive_joint";
  const std::string JOINTS_DRIVE_REAR  = "rear_drive_joint";
  if (joint_state) {
    for (size_t i = 0; i < joint_state->name.size(); ++i) {
      const auto &name = joint_state->name[i];
      if (name == JOINTS_STEER_FRONT) {
        cur_th_f = joint_state->position[i];
      } else if (name == JOINTS_STEER_REAR) {
        cur_th_r = joint_state->position[i];
      } else if (name == JOINTS_DRIVE_FRONT) {
        cur_v_f = joint_state->velocity[i];
      } else if (name == JOINTS_DRIVE_REAR) {
        cur_v_r = joint_state->velocity[i];
      }
    }
  }
  // 속도 그래프에 진동이 있어서 fillter하여 사용
  double alpha = 0.9;
  cur_v_f_filt_ = alpha * cur_v_f_filt_ + (1.0 - alpha) * cur_v_f;
  cur_v_r_filt_ = alpha * cur_v_r_filt_ + (1.0 - alpha) * cur_v_r;
  // [-90,90]
  auto normalize_angle = [](double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };
  // 각도 오차 & 상태 판단
  double err_f = normalize_angle(th_f_target - cur_th_f);
  double err_r = normalize_angle(th_r_target - cur_th_r);

  // 모드 판단
  const double ANGLE_TOL  = 0.5;  // rad 0.05
  const double SPEED_TOL  = 0.1;  // rad/s empty: 0.02 test2: 0.05
  const double T_STEER    = smooth_time_;   // 전체 정렬에 쓸 시간 (예: 1.0s)
  const double DT         = control_period_;
  bool is_misaligned = (std::fabs(err_f) > ANGLE_TOL) || (std::fabs(err_r) > ANGLE_TOL);
  // bool is_robot_moving = (std::fabs(cur_v_f_filt_) > SPEED_TOL) || (std::fabs(cur_v_r_filt_) > SPEED_TOL);;

  const int STOP_CONFIRM_CNT = 10; // 제어주기 100Hz면 0.1초 정도
  // WAIT_STOP모드에서 STOP 판단:
  if (std::abs(cur_v_f_filt_) < SPEED_TOL &&
      std::abs(cur_v_r_filt_) < SPEED_TOL){
      stop_confirm_counter_++;

  } else {
      stop_confirm_counter_ = 0;
  }
  bool really_stopped = (stop_confirm_counter_ >= STOP_CONFIRM_CNT);

  switch (mode_) {
  case SteerMode::DRIVE:
    if (is_misaligned) {
      // 조향이 틀어졌으면 → 먼저 멈추러 감
      mode_ = SteerMode::WAIT_STOP;
    }
    break;

  case SteerMode::WAIT_STOP:
    // 충분히 멈췄으면 이제 조향만 맞추는 단계로
    if (really_stopped) {
      mode_ = SteerMode::STEER_ALIGN;
    }
    // (사용자가 cmd_vel을 0으로 돌려서 더 이상 misaligned 아니면 바로 DRIVE로 복귀해도 됨)
    break;

  case SteerMode::STEER_ALIGN:
    // 각도가 tolerance 안으로 들어오면 다시 주행
    if (!is_misaligned) {
      mode_ = SteerMode::DRIVE;
    }
    break;
  }

  // cubic helper: 정지 상태에서만 사용
  auto cubic_step_once = [&](double cur_th, double target_th, double dt, double T) {
    double diff = normalize_angle(target_th - cur_th);
    double a2 = 3.0 * diff / (T * T);
    double a3 = -2.0 * diff / (T * T * T);

    double t = dt;
    double step = a2 * t * t + a3 * t * t * t;
    return cur_th + step;
  };
  // DRIVE 모드에서 쓸 파라미터
  const double Kp_drive            = 0.4;   // 조향 에러에 대한 비례 이득
  const double MAX_STEER_RATE_DRIVE = 0.3;  // 주행 중 허용 조향 속도 [rad/s]
  // ============= Stop – Steer – Go =============

  double th_f_cmd = cur_th_f;
  double th_r_cmd = cur_th_r;
  double v_f_cmd  = 0.0;
  double v_r_cmd  = 0.0;
  switch (mode_) {
    case SteerMode::DRIVE: {
      // --- 작은 에러만 남아있는 상태에서 부드럽게 보정 ---
      auto drive_steer_step = [&](double cur_th, double err) {
        // 1) 기본적으로는 P 제어
        double step = Kp_drive * err;   // rad

        // 2) 한 주기당 최대 조향 변화량 제한
        double max_step = MAX_STEER_RATE_DRIVE * DT;  // rad
        if (step >  max_step) step =  max_step;
        if (step < -max_step) step = -max_step;

        return cur_th + step;
      };

      th_f_cmd = drive_steer_step(cur_th_f, err_f);
      th_r_cmd = drive_steer_step(cur_th_r, err_r);

      // 구동은 바로 목표 속도로
      v_f_cmd  = v_f_target;
      v_r_cmd  = v_r_target;
      // std::cout << "DRIVE" << std::endl;
      break;
    }

    case SteerMode::WAIT_STOP:
      v_f_cmd  = 0.0;
      v_r_cmd  = 0.0;
      th_f_cmd = cur_th_f;
      th_r_cmd = cur_th_r;
      // std::cout << "STOP" << std::endl;
      break;

    case SteerMode::STEER_ALIGN:
      v_f_cmd  = 0.0;
      v_r_cmd  = 0.0;
      th_f_cmd = cubic_step_once(cur_th_f, th_f_target, DT, T_STEER);
      th_r_cmd = cubic_step_once(cur_th_r, th_r_target, DT, T_STEER);
      // std::cout << "STEER_ALIGN" << std::endl;
      break;
  }


  // 3. JointTrajectory 메시지 생성 (position-only, velocity는 안 보냄)
  steer_traj = trajectory_msgs::msg::JointTrajectory{};
  steer_traj.joint_names = steer_joints;
  steer_traj.points.resize(1);
  auto &pt = steer_traj.points[0];
  pt.positions = {th_f_cmd, th_r_cmd};
  pt.time_from_start = rclcpp::Duration::from_seconds(0.0);

  drive_cmd = std_msgs::msg::Float64MultiArray{};
  drive_cmd.data = {v_f_cmd, v_r_cmd};
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