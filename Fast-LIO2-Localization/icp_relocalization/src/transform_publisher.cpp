#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 이 헤더가 필수입니다!

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
        : Node("transform_publisher_node")
    {
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_footprint");
        this->declare_parameter<std::string>("map_frame_id", "map");

        this->get_parameter("odom_frame_id", odom_frame_id_);
        this->get_parameter("base_frame_id", base_frame_id_);
        this->get_parameter("map_frame_id", map_frame_id_);

        // TF Listener 초기화
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Static Broadcaster 초기화
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // ICP 결과 구독
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Static Transform Publisher Node Started. Waiting for /icp_result...");
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (is_initialized_) {
            return; 
        }

        // 1. ICP 결과 (Map -> Base) 변환
        tf2::Transform transform_map_to_base;
        tf2::fromMsg(msg->pose.pose, transform_map_to_base);

        // 2. 현재 오도메트리 (Odom -> Base) 조회
        // [중요] 여기 타입이 tf2::Transform 이어야 합니다! (Stamped 아님)
        tf2::Transform transform_odom_to_base; 
        try {
            geometry_msgs::msg::TransformStamped tf_msg;
            // 최신 TF 조회 (타임아웃 1.0초)
            tf_msg = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
            
            // Transform -> Transform 변환 (성공)
            tf2::fromMsg(tf_msg.transform, transform_odom_to_base);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Initial Odom TF not found: %s", ex.what());
            return; 
        }

        // 3. Map -> Odom 계산 (핵심 공식)
        // T_map_odom = T_map_base * (T_odom_base)^-1
        tf2::Transform transform_map_to_odom = transform_map_to_base * transform_odom_to_base.inverse();

        // 4. Static TF 발행
        geometry_msgs::msg::TransformStamped tf_out;
        tf_out.header.stamp = rclcpp::Time(0);
        tf_out.header.frame_id = map_frame_id_;
        tf_out.child_frame_id = odom_frame_id_;
        tf_out.transform = tf2::toMsg(transform_map_to_odom);

        static_broadcaster_->sendTransform(tf_out);
        
        is_initialized_ = true; // 초기화 완료 플래그 설정
        RCLCPP_INFO(this->get_logger(), "Static TF (map->odom) published based on ICP result!");
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string map_frame_id_;
    bool is_initialized_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
    rclcpp::shutdown();
    return 0;
}