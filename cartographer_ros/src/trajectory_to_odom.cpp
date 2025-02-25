#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class TrackedPoseToOdom : public rclcpp::Node
{
public:
    TrackedPoseToOdom()
        : Node("tracked_pose_to_odom"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // /tracked_pose 구독자 생성
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 20, std::bind(&TrackedPoseToOdom::callback, this, _1));

        // /odom 퍼블리셔 생성
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);

        // use_sim_time 파라미터 확인
        if (!this->get_parameter("use_sim_time", use_sim_time_)) {
            RCLCPP_INFO(this->get_logger(), "\033[33muse_sim_time NOT SET. Defaulting to false.\033[0m");
            use_sim_time_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "TrackedPoseToOdom node initialized.");
    }

private:
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try
        {
            // map -> base_link 변환 획득
            auto map_to_odom = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            // Odometry 메시지 생성
            nav_msgs::msg::Odometry odom_msg;

            // 타임스탬프 설정 (use_sim_time에 따라)
            builtin_interfaces::msg::Time current_time;
            current_time.sec = this->get_clock()->now().seconds();
            current_time.nanosec = this->get_clock()->now().nanoseconds() % 1000000000;
            odom_msg.header.stamp = use_sim_time_ ? current_time : msg->header.stamp;

            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_link";

            // transform 데이터를 이용하여 위치 및 자세 설정
            odom_msg.pose.pose.position.x = map_to_odom.transform.translation.x;
            odom_msg.pose.pose.position.y = map_to_odom.transform.translation.y;
            odom_msg.pose.pose.position.z = 0.0;

            odom_msg.pose.pose.orientation = map_to_odom.transform.rotation;

            // 바로 odom 메시지 publish
            publisher_->publish(odom_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool use_sim_time_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedPoseToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
