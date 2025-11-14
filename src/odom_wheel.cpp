#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <thread>
#include <chrono>

class OdometryEstimator : public rclcpp::Node
{
public:
    OdometryEstimator()
        : Node("odometry_estimator"),
          x_(0.0), y_(0.0), yaw_(0.0),
          left_wheel_vel_(0.0), right_wheel_vel_(0.0)
    {
        // Load parameters
        this->declare_parameter<double>("wheel_radius", 0.08);
        this->declare_parameter<double>("wheel_base", 0.30);
        this->declare_parameter<double>("update_rate", 20.0);
        this->declare_parameter<double>("rotation_time_at_half_vel", 0.375);
        this->declare_parameter<double>("cmd_vel_test_value", 0.5);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_   = this->get_parameter("wheel_base").as_double();
        update_rate_  = this->get_parameter("update_rate").as_double();
        double rotation_time = this->get_parameter("rotation_time_at_half_vel").as_double();
        double cmd_vel_test = this->get_parameter("cmd_vel_test_value").as_double();

        // Calculate conversion factor at runtime:
        // At cmd_vel = cmd_vel_test, 1 rotation (2π * wheel_radius) occurs in rotation_time
        // Linear velocity = (2π * wheel_radius) / rotation_time
        // Conversion factor = linear_velocity / cmd_vel_test
        cmd_vel_to_wheel_vel_factor_ = (2.0 * M_PI * wheel_radius_) / (cmd_vel_test * rotation_time);

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_executed", 10,
            std::bind(&OdometryEstimator::cmdVelCallback, this, std::placeholders::_1));

        // Publisher with default QoS (Reliable, Volatile)
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Publisher created for /odom_wheel (actual topic: %s)", 
                   odom_pub_->get_topic_name());

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        last_time_ = this->now();

        // Timer - ensure update_rate is valid
        if (update_rate_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid update_rate: %.2f, using default 20.0 Hz", update_rate_);
            update_rate_ = 20.0;
        }
        auto period = std::chrono::duration<double>(1.0 / update_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&OdometryEstimator::updateOdometry, this));
        
        RCLCPP_INFO(this->get_logger(), "Odometry estimator started. Publishing to /odom_wheel at %.1f Hz", update_rate_);
    }
    
    // Public method to publish odometry (called from main and timer)
    void publishOdometry()
    {
        rclcpp::Time current_time = this->now();
        
        // Always compute current velocities for publishing
        double v = (right_wheel_vel_ + left_wheel_vel_) / 2.0;
        double omega = (right_wheel_vel_ - left_wheel_vel_) / wheel_base_;

        // Publish TF
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = 0.0;
        odom_tf.transform.rotation.y = 0.0;
        odom_tf.transform.rotation.z = std::sin(yaw_ / 2.0);
        odom_tf.transform.rotation.w = std::cos(yaw_ / 2.0);

        tf_broadcaster_->sendTransform(odom_tf);

        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = omega;

        // Verify publisher is valid before publishing
        if (!odom_pub_) {
            RCLCPP_ERROR(this->get_logger(), "Publisher is null! Cannot publish odometry.");
            return;
        }
        
        try {
            odom_pub_->publish(odom_msg);
            
            // Debug: log first few messages to verify publishing
            static int publish_count = 0;
            if (publish_count++ < 3) {
                RCLCPP_INFO(this->get_logger(), "Published odometry message #%d: x=%.3f, y=%.3f, yaw=%.3f, v=%.3f, omega=%.3f", 
                            publish_count, x_, y_, yaw_, v, omega);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while publishing: %s", e.what());
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Convert cmd_vel (scaled 0-1.0) to actual wheel linear velocities
        // Scale the linear velocity using the experimental conversion factor
        double v_scaled = msg->linear.x * cmd_vel_to_wheel_vel_factor_;
        double omega = msg->angular.z;

        left_wheel_vel_  = v_scaled - omega * (wheel_base_ / 2.0);
        right_wheel_vel_ = v_scaled + omega * (wheel_base_ / 2.0);
    }

    void updateOdometry()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        // Skip integration if dt is too small or invalid
        if (dt > 0.0 && dt < 1.0) {
            // Compute robot linear and angular velocity from wheels
            double v = (right_wheel_vel_ + left_wheel_vel_) / 2.0;
            double omega = (right_wheel_vel_ - left_wheel_vel_) / wheel_base_;

            // Integrate pose
            x_ += v * std::cos(yaw_) * dt;
            y_ += v * std::sin(yaw_) * dt;
            yaw_ += omega * dt;
        }

        publishOdometry();
        last_time_ = current_time;
    }

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double wheel_radius_;
    double wheel_base_;
    double update_rate_;
    double cmd_vel_to_wheel_vel_factor_;

    // Robot state
    double x_, y_, yaw_;
    double left_wheel_vel_, right_wheel_vel_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryEstimator>();
    
    // Publish an initial message after node is created to ensure topic is advertised
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    node->publishOdometry();
    
    // Use standard spin - MultiThreadedExecutor may cause issues
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

