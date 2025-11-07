#pragma once
#include <string>
#include <vector>

namespace tr_dual_steering {

struct ModuleSpec {
  std::string name;        // e.g., "front", "rear" (임의)
  std::string steer_joint; // steering joint name
  std::string drive_joint; // wheel rotation joint name
  double x{0.0};           // [m] base_link 좌표계에서 모듈 위치 (앞 +x, 좌 +y)
  double y{0.0};
  double steer_angle{0.0}; // [rad] 마지막 명령
  double drive_pos{0.0};   // [rad] 적분된 바퀴 각도
};

struct Params {
  double wheel_diameter{0.1};        // [m]
  double publish_rate_hz{50.0};    // [Hz]
  std::string cmd_vel_topic{"/cmd_vel"};
  std::string traj_topic{"/joint_trajectory"};
};

} // namespace tr_dual_steering
