#ifndef TANK_ODOM__TANK_ODOM_NODE_HPP_
#define TANK_ODOM__TANK_ODOM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tank_interface/msg/motor_rpm_array.hpp>

namespace tank_odom {

class TankOdomNode : public rclcpp::Node {
public:
    TankOdomNode();

private:
    // Parameters
    double front_wheel_diameter_;
    double wheel_separation_;
    int odom_frequency_;
    bool publish_tf_;

    // State variables
    double x_;
    double y_;
    double theta_;
    geometry_msgs::msg::Twist current_twist_;
    rclcpp::Time last_odom_time_;
    double left_track_velocity_;   // Added for track velocity
    double right_track_velocity_;  // Added for track velocity

    // Subscribers and Publishers
    rclcpp::Subscription<tank_interface::msg::MotorRPMArray>::SharedPtr wheel_rpm_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr actual_speed_pub_;  // Added publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // Methods
    void initializeParameters();
    void setupSubscribersAndPublishers();
    void wheelRPMCallback(const tank_interface::msg::MotorRPMArray::SharedPtr msg);
    void publishOdometry();
    void publishActualSpeed();  // Added method
    void publishTransform(const nav_msgs::msg::Odometry& odom_msg);
    double rpmToVelocity(double rpm, double wheel_diameter) const;
    void updateOdometry(const tank_interface::msg::MotorRPMArray::SharedPtr msg);
};

} // namespace tank_odom

#endif // TANK_ODOM__TANK_ODOM_NODE_HPP_