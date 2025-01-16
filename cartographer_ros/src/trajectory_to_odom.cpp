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
        // Subscriber to /tracked_pose
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 10, std::bind(&TrackedPoseToOdom::callback, this, _1));

        // Publisher for /odom
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Timer for periodic /odom publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrackedPoseToOdom::publish_last_odom, this));

        // Parameter for use_sim_time
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
            // Get the transform from map to odom
            geometry_msgs::msg::TransformStamped map_to_odom = tf_buffer_.lookupTransform(
                "odom", "map", tf2::TimePointZero);

            // Transform the tracked pose from map to odom frame
            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(*msg, transformed_pose, map_to_odom);

            // Extract position and orientation from transformed pose
            double position_x = transformed_pose.pose.position.x;
            double position_y = transformed_pose.pose.position.y;
            double orientation_z = transformed_pose.pose.orientation.z;
            double orientation_w = transformed_pose.pose.orientation.w;

            // Create Odometry message
            nav_msgs::msg::Odometry odom_msg;

            // Set timestamp based on use_sim_time
            builtin_interfaces::msg::Time current_time;
            current_time.sec = this->get_clock()->now().seconds();
            current_time.nanosec = this->get_clock()->now().nanoseconds() % 1000000000;
            odom_msg.header.stamp = use_sim_time_ ? current_time : msg->header.stamp;

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

            RCLCPP_INFO(this->get_logger(), "Published /odom with transformed pose");
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        }
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
    std::shared_ptr<nav_msgs::msg::Odometry> last_odom_msg_;

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
