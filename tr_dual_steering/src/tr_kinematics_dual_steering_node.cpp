#include "../include/SteerDrive2WKinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class PlatformCtrlNode: public rclcpp::Node
{
public:
	PlatformCtrlNode(): Node("tr_dual_steering_node") {}

	int init() {
        this->declare_parameter<std::string>("odom_frame", "odom");
		this->declare_parameter<std::string>("base_frame", "base_footprint");
		this->declare_parameter<double>("wheel_diameter", 0.2);
		this->declare_parameter<double>("publish_rate_hz", 1000);
		this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
		this->declare_parameter<std::string>("steer_traj_topic", "/joint_trajectory_controller/joint_trajectory");
    	this->declare_parameter<std::string>("drive_cmd_topic",  "/velocity_controller/commands");
		this->declare_parameter<std::vector<std::string>>("steer_joints", {"front_steer_joint","rear_steer_joint"});
		this->declare_parameter<std::vector<std::string>>("drive_joints", {"front_drive_joint","rear_drive_joint"});
		this->declare_parameter<std::vector<double>>("module_x", {+0.46, -0.46});
		this->declare_parameter<std::vector<double>>("module_y", {0.30, -0.30});

        this->get_parameter("odom_frame", odom_frame);
		this->get_parameter("base_frame", base_frame);
        this->get_parameter("wheel_diameter", wheel_diameter);
        this->get_parameter("publish_rate_hz", publish_rate);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);
        this->get_parameter("steer_traj_topic", steer_traj_topic);
    	this->get_parameter("drive_cmd_topic",  drive_cmd_topic);
        this->get_parameter("steer_joints", steer_joints);
        this->get_parameter("drive_joints", drive_joints);
        this->get_parameter("module_x", module_x);
        this->get_parameter("module_y", module_y);

		topicPub_Odometry = this->create_publisher<nav_msgs::msg::Odometry>("/odom", publish_rate);
		pubSteer_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(steer_traj_topic, publish_rate);
    	pubDrive_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(drive_cmd_topic, publish_rate);

		topicSub_ComVel = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, 1, std::bind(&PlatformCtrlNode::receiveCmd, this, _1));
		topicSub_DriveState = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&PlatformCtrlNode::receiveOdo, this, _1));

		odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		this->kin = new SteerDrive2WKinematics();
		this->kin->setModuleSpec(wheel_diameter, steer_joints, drive_joints, module_x, module_y);

		return 0;
	}

	void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr twist) {
		trajectory_msgs::msg::JointTrajectory steer_traj;
		std_msgs::msg::Float64MultiArray drive_cmd;

		kin->execInvKin(twist, steer_traj, drive_cmd);

		steer_traj.header.stamp = {};
		pubSteer_->publish(steer_traj);
		pubDrive_->publish(drive_cmd);
	}

	void receiveOdo(const sensor_msgs::msg::JointState::SharedPtr js) {

		nav_msgs::msg::Odometry odom;
		odom.header.frame_id = odom_frame;
		odom.child_frame_id = base_frame; // 원래 base_link인데 base_footprint로 변경
		kin->execForwKin(js, odom);
		topicPub_Odometry->publish(odom);

		//odometry transform:
		if(sendTransform) {
			geometry_msgs::msg::TransformStamped odom_trans;
			odom_trans.header.stamp = odom.header.stamp;
            odom_trans.header.frame_id = odom_frame;
            odom_trans.child_frame_id = base_frame;

			odom_trans.transform.translation.x = odom.pose.pose.position.x;
			odom_trans.transform.translation.y = odom.pose.pose.position.y;
			odom_trans.transform.translation.z = odom.pose.pose.position.z;
			odom_trans.transform.rotation = odom.pose.pose.orientation;
			odom_broadcaster->sendTransform(odom_trans);
		}
	}

private:
	SteerDrive2WKinematics* kin = 0;
    bool sendTransform = true;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topicPub_Odometry;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pubSteer_;
  	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubDrive_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_ComVel;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topicSub_DriveState;
	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

	double wheel_diameter = 0.0;
	double publish_rate = 0.0;
	std::vector<double> module_x = {0.0, 0.0};
    std::vector<double> module_y = {0.0, 0.0};

	std::string cmd_vel_topic;
	std::string steer_traj_topic;
	std::string  drive_cmd_topic;
    std::string odom_frame;
    std::string base_frame;

    std::vector<std::string> steer_joints;
    std::vector<std::string> drive_joints;
};

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = std::make_shared<PlatformCtrlNode>();
	if(nh->init() != 0) {
			RCLCPP_ERROR_STREAM(nh->get_logger(),"tr_kinematics_dual_steering_node: init failed!");
		}
	rclcpp::spin(nh);
	
	return 0;
}