#include "tank_odom/tank_odom_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace tank_odom {

TankOdomNode::TankOdomNode() 
    : Node("tank_odom")
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , left_track_velocity_(0.0)
    , right_track_velocity_(0.0) {
    
    initializeParameters();
    setupSubscribersAndPublishers();

    last_odom_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Tank Odometry Node initialized");
    RCLCPP_INFO(this->get_logger(), "Front wheel diameter: %.3f m", front_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "Odom Frequency: %d Hz", odom_frequency_);
    RCLCPP_INFO(this->get_logger(), "Publish tf: %s", publish_tf_ ? "true" : "false");
}

void TankOdomNode::initializeParameters() {
    // Physical parameters
    this->declare_parameter("front_wheel_diameter", 0.186);
    this->declare_parameter("wheel_separation", 0.515);
    this->declare_parameter("odom_frequency", 20);
    this->declare_parameter("publish_tf", true);

    // Load parameters
    front_wheel_diameter_ = this->get_parameter("front_wheel_diameter").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    odom_frequency_ = this->get_parameter("odom_frequency").as_int();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
}

void TankOdomNode::setupSubscribersAndPublishers() {
    // Set up subscriber for wheel RPM
    wheel_rpm_sub_ = this->create_subscription<tank_interface::msg::MotorRPMArray>(
        "/actual_wheel_speed", 30, 
        std::bind(&TankOdomNode::wheelRPMCallback, this, std::placeholders::_1));

    // Set up odometry publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Set up actual speed publisher
    actual_speed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/actual_tank_speed", 10);

    // Set up transform broadcaster if enabled
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Set up odometry timer
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/odom_frequency_)),
        std::bind(&TankOdomNode::publishOdometry, this));
}

double TankOdomNode::rpmToVelocity(double rpm, double wheel_diameter) const {
    return (rpm * M_PI * wheel_diameter) / 60.0;
}

void TankOdomNode::wheelRPMCallback(const tank_interface::msg::MotorRPMArray::SharedPtr msg) {
    updateOdometry(msg);
}

void TankOdomNode::updateOdometry(const tank_interface::msg::MotorRPMArray::SharedPtr msg) {
    auto current_time = this->now();
    double dt = (current_time - last_odom_time_).seconds();
    last_odom_time_ = current_time;

    if (dt == 0.0) return;

    // Calculate linear velocities for left and right tracks using front wheels
    left_track_velocity_ = rpmToVelocity(msg->rpm[0], front_wheel_diameter_);
    right_track_velocity_ = rpmToVelocity(msg->rpm[1], front_wheel_diameter_);

    // Calculate robot's linear and angular velocity
    double linear_vel = (right_track_velocity_ + left_track_velocity_) / 2.0;
    double angular_vel = (right_track_velocity_ - left_track_velocity_) / wheel_separation_;

    // Update pose
    double delta_theta = (angular_vel * dt)/3;
    double delta_x, delta_y;

    if (std::abs(angular_vel) < 0.001) {
        // Straight line motion
        delta_x = linear_vel * cos(theta_) * dt;
        delta_y = linear_vel * sin(theta_) * dt;
    } else {
        // Arc motion
        double radius = linear_vel / angular_vel;
        delta_x = -radius * sin(theta_) + radius * sin(theta_ + delta_theta);
        delta_y = radius * cos(theta_) - radius * cos(theta_ + delta_theta);
    }

    // Update position and orientation
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Normalize theta
    theta_ = std::fmod(theta_, 2 * M_PI);
    if (theta_ > M_PI) theta_ -= 2 * M_PI;
    if (theta_ < -M_PI) theta_ += 2 * M_PI;

    // Update current twist
    current_twist_.linear.x = linear_vel;
    current_twist_.angular.z = angular_vel;

    // Publish actual speed immediately after calculation
    publishActualSpeed();

    RCLCPP_DEBUG(this->get_logger(), "Current Velocity - Linear: %.2f m/s, Angular: %.2f rad/s", linear_vel, angular_vel);
    RCLCPP_DEBUG(this->get_logger(), "Track Velocities - Left: %.2f m/s, Right: %.2f m/s", left_track_velocity_, right_track_velocity_);
}

void TankOdomNode::publishActualSpeed() {
    auto twist_msg = geometry_msgs::msg::Twist();
    
    // Set the actual velocities
    twist_msg.linear.x = current_twist_.linear.x;  // Forward velocity
    twist_msg.angular.z = current_twist_.angular.z;  // Angular velocity
    
    actual_speed_pub_->publish(twist_msg);
}

void TankOdomNode::publishOdometry() {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Set position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Set orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Set velocities
    odom_msg.twist.twist = current_twist_;

    // Set covariance
    for (size_t i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;  // x
    odom_msg.pose.covariance[7] = 0.01;  // y
    odom_msg.pose.covariance[35] = 0.01; // theta
    
    odom_msg.twist.covariance[0] = 0.01;  // linear x velocity
    odom_msg.twist.covariance[35] = 0.01; // angular z velocity

    // Publish odometry message
    odom_pub_->publish(odom_msg);

    // Publish transform if enabled
    if (publish_tf_) {
        publishTransform(odom_msg);
    }
}

void TankOdomNode::publishTransform(const nav_msgs::msg::Odometry& odom_msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_msg.header;
    transform.child_frame_id = odom_msg.child_frame_id;
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = odom_msg.pose.pose.position.z;
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
}

} // namespace tank_odom

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tank_odom::TankOdomNode>());
    rclcpp::shutdown();
    return 0;
}