#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class TrackedPoseToOdom : public rclcpp::Node
{
public:
    TrackedPoseToOdom()
        : Node("tracked_pose_to_odom"), static_tf_broadcaster_(*this)
    {
        // Subscriber to /tracked_pose
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 10, std::bind(&TrackedPoseToOdom::callback, this, _1));

        // Publisher for /odom
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Timer for periodic /odom publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrackedPoseToOdom::publish_last_odom, this));

        // Static transform: odom -> base_link
        publish_static_transform();

        RCLCPP_INFO(this->get_logger(), "TrackedPoseToOdom node initialized.");
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "odom";       // Parent frame
        static_transform.child_frame_id = "base_link";   // Child frame

        // Translation (fixed at 0,0,0)
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;

        // Rotation (fixed at 0,0,0,1)
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        static_tf_broadcaster_.sendTransform(static_transform);
        RCLCPP_INFO(this->get_logger(), "Published static transform: odom -> base_link.");
    }

    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Extract position and orientation from /tracked_pose
        double position_x = msg->pose.position.x;
        double position_y = msg->pose.position.y;
        double orientation_z = msg->pose.orientation.z;
        double orientation_w = msg->pose.orientation.w;

        // Calculate heading from orientation quaternion
        double heading = atan2(2.0 * (orientation_w * orientation_z), 1.0 - 2.0 * (orientation_z * orientation_z));

        // Log the current data from /tracked_pose
        RCLCPP_INFO(this->get_logger(), "TrackedPose -> Raw Position: (%.2f, %.2f), Orientation (Z: %.2f, W: %.2f)",
                    position_x, position_y, orientation_z, orientation_w);

        // Log the computed heading
        RCLCPP_INFO(this->get_logger(), "Computed Heading: %.2f radians", heading);

        // Create Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = position_x;
        odom_msg.pose.pose.position.y = position_y;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = orientation_z;
        odom_msg.pose.pose.orientation.w = orientation_w;

        // Save the last odometry message
        last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(odom_msg);
        publisher_->publish(odom_msg);

        RCLCPP_INFO(this->get_logger(), "Published /odom: Position (%.2f, %.2f), Heading %.2f radians",
                    position_x, position_y, heading);
    }

    void publish_last_odom()
    {
        if (last_odom_msg_)
        {
            publisher_->publish(*last_odom_msg_);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    std::shared_ptr<nav_msgs::msg::Odometry> last_odom_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedPoseToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
