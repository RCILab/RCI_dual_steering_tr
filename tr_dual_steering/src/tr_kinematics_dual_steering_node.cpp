#include "../include/SteerDrive2WKinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>

using std::placeholders::_1;

class PlatformCtrlNode : public rclcpp::Node
{
public:
  PlatformCtrlNode() : Node("tr_dual_steering_node") {}

  int init()
  {
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<double>("wheel_diameter", 0.32);
    this->declare_parameter<double>("publish_rate_hz", 100.0);
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("steer_traj_topic", "/joint_trajectory_controller/joint_trajectory");
    this->declare_parameter<std::string>("drive_cmd_topic", "/velocity_controller/commands");
    this->declare_parameter<std::vector<std::string>>("steer_joints", {"front_steer_joint", "rear_steer_joint"});
    this->declare_parameter<std::vector<std::string>>("drive_joints", {"front_drive_joint", "rear_drive_joint"});
    this->declare_parameter<std::vector<double>>("module_x", {+0.46, -0.46});
    this->declare_parameter<std::vector<double>>("module_y", {0.30, -0.30});

    // timeout 파라미터만 사용 (가속도 rate limit은 일단 execInvKin에 맡기자)
    this->declare_parameter<double>("cmd_timeout_sec", 8.0);  // 8초

    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("wheel_diameter", wheel_diameter_);
    this->get_parameter("publish_rate_hz", publish_rate_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
    this->get_parameter("steer_traj_topic", steer_traj_topic_);
    this->get_parameter("drive_cmd_topic", drive_cmd_topic_);
    this->get_parameter("steer_joints", steer_joints_);
    this->get_parameter("drive_joints", drive_joints_);
    this->get_parameter("module_x", module_x_);
    this->get_parameter("module_y", module_y_);

    this->get_parameter("cmd_timeout_sec", cmd_timeout_sec_);
    cmd_timeout_ = rclcpp::Duration::from_seconds(cmd_timeout_sec_);

    RCLCPP_INFO(this->get_logger(), "wheel_diameter: %.3f", wheel_diameter_);

    topicPub_Odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom1", 10);
    pubSteer_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(steer_traj_topic_, 10);
    pubDrive_ =this->create_publisher<std_msgs::msg::Float64MultiArray>(drive_cmd_topic_, 10);

    topicSub_ComVel_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1,std::bind(&PlatformCtrlNode::receiveCmd, this, _1));
    topicSub_DriveState_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,std::bind(&PlatformCtrlNode::receiveOdo, this, _1));

    // odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    kin_ = new SteerDrive2WKinematics();
    kin_->setModuleSpec(wheel_diameter_, steer_joints_, drive_joints_, module_x_, module_y_);

    // execInvKin 내부에서 쓰는 control_period_와 publish_rate를 맞춰주고 싶으면
    // kin_->setControlPeriod(1.0 / publish_rate_);  // 이런 setter를 만들어 써도 좋음

    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "publish_rate_hz <= 0.0, set to 100 Hz");
      publish_rate_ = 100.0;
    }

    const double period_sec = 1.0 / publish_rate_;
    last_cmd_time_ = this->now();

    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&PlatformCtrlNode::controlLoop, this));

    return 0;
  }

  // ====== /cmd_vel 콜백 : 목표 Twist만 저장 ======
  void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_cmd_ = *twist;
    last_cmd_time_ = this->now();
    has_cmd_ = true;
  }

  // ====== /joint_states 콜백 : 최신 관절 상태 저장 + odom 갱신 ======
  void receiveOdo(const sensor_msgs::msg::JointState::SharedPtr js)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_joint_state_ = js;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;  // base_link 대신 base_footprint

    kin_->execForwKin(js, odom);
    topicPub_Odometry_->publish(odom);

    // // TF까지 보내고 싶으면 여기 주석 풀기
    // if (sendTransform_) {
    //   geometry_msgs::msg::TransformStamped odom_trans;
    //   odom_trans.header.stamp = odom.header.stamp;
    //   odom_trans.header.frame_id = odom_frame_;
    //   odom_trans.child_frame_id = base_frame_;
    //
    //   odom_trans.transform.translation.x = odom.pose.pose.position.x;
    //   odom_trans.transform.translation.y = odom.pose.pose.position.y;
    //   odom_trans.transform.translation.z = odom.pose.pose.position.z;
    //   odom_trans.transform.rotation = odom.pose.pose.orientation;
    //   odom_broadcaster_->sendTransform(odom_trans);
    // }
  }

private:
  // ====== 타이머 기반 제어 루프 ======
  void controlLoop()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // joint_state를 아직 못 받았으면 아무 것도 하지 않음
    if (!latest_joint_state_) {
      return;
    }

    auto now = this->now();

    geometry_msgs::msg::Twist cmd_for_this_tick;

    // cmd_vel을 한 번도 못 받았거나, timeout이 지나면 → 0 명령
    if (!has_cmd_ || (now - last_cmd_time_) > cmd_timeout_) {
      // 기본 생성자이면 0으로 초기화됨
      cmd_for_this_tick = geometry_msgs::msg::Twist();
    } else {
      // 마지막으로 받은 /cmd_vel 그대로 사용
      cmd_for_this_tick = target_cmd_;
    }

    // 이 틱에서 사용할 Twist로 execInvKin 한 스텝 수행
    trajectory_msgs::msg::JointTrajectory steer_traj;
    std_msgs::msg::Float64MultiArray drive_cmd;

    auto cmd_ptr =
      std::make_shared<geometry_msgs::msg::Twist>(cmd_for_this_tick);
    kin_->execInvKin(cmd_ptr, latest_joint_state_, steer_traj, drive_cmd);

    steer_traj.header.stamp = rclcpp::Time(0);
    pubSteer_->publish(steer_traj);
    pubDrive_->publish(drive_cmd);
  }

private:
  SteerDrive2WKinematics * kin_ = nullptr;
  bool sendTransform_ = true;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topicPub_Odometry_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pubSteer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubDrive_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_ComVel_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topicSub_DriveState_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_ = nullptr;

  double wheel_diameter_ = 0.0;
  double publish_rate_   = 0.0;
  std::vector<double> module_x_ = {0.0, 0.0};
  std::vector<double> module_y_ = {0.0, 0.0};

  std::string cmd_vel_topic_;
  std::string steer_traj_topic_;
  std::string drive_cmd_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  std::vector<std::string> steer_joints_;
  std::vector<std::string> drive_joints_;

  // controlLoop & timeout 관련 상태
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::mutex mutex_;

  geometry_msgs::msg::Twist target_cmd_;  // 마지막 /cmd_vel
  bool has_cmd_ = false;

  double cmd_timeout_sec_ = 8.0;
  rclcpp::Duration cmd_timeout_{0, 0};

  rclcpp::Time last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<PlatformCtrlNode>();
  if (nh->init() != 0) {
    RCLCPP_ERROR_STREAM(
      nh->get_logger(),
      "tr_kinematics_dual_steering_node: init failed!");
  }
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
