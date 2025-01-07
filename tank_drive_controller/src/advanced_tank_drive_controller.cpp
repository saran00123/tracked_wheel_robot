#include "tank_drive_controller/advanced_tank_drive_controller.hpp"

namespace tank_controller {

AdvancedTankController::AdvancedTankController() 
    : Node("advanced_tank_controller")
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0) {
    
    initializeParameters();
    setupSubscribersAndPublishers();
    setupTimers();

    last_cmd_time_ = this->now();
    last_odom_time_ = this->now();
    wheel_diameter_ratio_ = front_wheel_diameter_ / rear_wheel_diameter_;

    front_wheel_left_target_rpm = 0.0;
    front_wheel_right_target_rpm = 0.0;

    RCLCPP_INFO(this->get_logger(), "Advanced Tank Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Front wheel diameter: %.3f m", front_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Rear wheel diameter: %.3f m", rear_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "Odom Frequency: %d m/s", odom_frequency_);
    RCLCPP_INFO(this->get_logger(), "Motor Command Frequency: %d rad/s", motor_command_frequency_);
    RCLCPP_INFO(this->get_logger(), "Use PID: %s ", use_pid ? "true" : "false" );
    RCLCPP_INFO(this->get_logger(), "Publish tf: %s ", publish_tf ? "true" : "false");

}

void AdvancedTankController::initializeParameters() {
    // Physical parameters (in meters)
    this->declare_parameter("front_wheel_diameter", 0.186);
    this->declare_parameter("rear_wheel_diameter", 0.148);
    this->declare_parameter("wheel_separation", 0.515);
    this->declare_parameter("wheel_base", 0.700);

    // Command limits
    this->declare_parameter("max_linear_velocity", 1.0);      // m/s
    this->declare_parameter("max_angular_velocity", 2.0);     // rad/s
    this->declare_parameter("max_linear_acceleration", 0.5);  // m/s^2
    this->declare_parameter("max_angular_acceleration", 1.0); // rad/s^2
    this->declare_parameter("max_rpm", 100.0);               // RPM
    this->declare_parameter("odom_frequency", 20);               // RPM
    this->declare_parameter("motor_command_frequency", 50);               // RPM

    // Add PID parameters for linear and angular control
    this->declare_parameter("linear_pid_kp", 1.0);
    this->declare_parameter("linear_pid_ki", 0.1);
    this->declare_parameter("linear_pid_kd", 0.05);
    this->declare_parameter("angular_pid_kp", 1.2);
    this->declare_parameter("angular_pid_ki", 0.1);
    this->declare_parameter("angular_pid_kd", 0.05);
    this->declare_parameter("pid_windup_limit", 1.0);

    // Load parameters
    front_wheel_diameter_ = this->get_parameter("front_wheel_diameter").as_double();
    rear_wheel_diameter_ = this->get_parameter("rear_wheel_diameter").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    max_linear_acceleration_ = this->get_parameter("max_linear_acceleration").as_double();
    max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
    max_rpm_ = this->get_parameter("max_rpm").as_double();
    odom_frequency_ = this->get_parameter("odom_frequency").as_int();
    motor_command_frequency_ = this->get_parameter("motor_command_frequency").as_int();
    
    linear_pid_kp_ = this->get_parameter("linear_pid_kp").as_double();
    linear_pid_ki_ = this->get_parameter("linear_pid_ki").as_double();
    linear_pid_kd_ = this->get_parameter("linear_pid_kd").as_double();
    angular_pid_kp_ = this->get_parameter("angular_pid_kp").as_double();
    angular_pid_ki_ = this->get_parameter("angular_pid_ki").as_double();
    angular_pid_kd_ = this->get_parameter("angular_pid_kd").as_double();
    pid_windup_limit_ = this->get_parameter("pid_windup_limit").as_double();
    
    // Initialize PID controllers
    linear_pid_ = std::make_unique<PIDController>(
        linear_pid_kp_, linear_pid_ki_, linear_pid_kd_,
        -max_linear_velocity_, max_linear_velocity_, pid_windup_limit_);
        
    angular_pid_ = std::make_unique<PIDController>(
        angular_pid_kp_, angular_pid_ki_, angular_pid_kd_,
        -max_angular_velocity_, max_angular_velocity_, pid_windup_limit_);


}

void AdvancedTankController::setupSubscribersAndPublishers() {
    // Setting up subscribers and publishers

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, 
        std::bind(&AdvancedTankController::cmdVelCallback, this, std::placeholders::_1));

    wheel_rpm_sub_ = this->create_subscription<tank_interface::msg::MotorRPMArray>(
        "/actual_wheel_speed", 30, 
        std::bind(&AdvancedTankController::wheelRPMCallback, this, std::placeholders::_1));


    motor_rpm_pub_ = this->create_publisher<tank_interface::msg::MotorRPMArray>("motor_rpms", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void AdvancedTankController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    // Command Velocity Callback function to handle incoming Twist messages
    RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel - linear: %.2f, angular: %.2f", msg->linear.x, msg->angular.z);

    // Clamp linear and angular velocities to limits     
    target_twist_.linear.x = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    target_twist_.angular.z = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    last_cmd_time_ = this->now();
                                       
    RCLCPP_DEBUG(this->get_logger(), "Target velocities after clamping - linear: %.2f, angular: %.2f", target_twist_.linear.x, target_twist_.angular.z);
}

// void AdvancedTankController::updateCommand() {
//     const double velocity_deadband = 0.1;  // 10 cm/s
//     const double angular_deadband = 0.01;   // ~0.6 degrees/s
    
//     // Get current time and calculate dt
//     auto current_time = this->now();
//     double dt = (current_time - last_cmd_time_).seconds();
    
//     // Create acceleration-limited twist
//     geometry_msgs::msg::Twist limited_twist = target_twist_;
    
//     // Calculate maximum allowed velocity changes based on acceleration limits
//     double max_linear_vel_change = max_linear_acceleration_ * dt;
//     double max_angular_vel_change = max_angular_acceleration_ * dt;

//     // Calculate desired velocity changes
//     double linear_vel_change = target_twist_.linear.x - actual_twist_.linear.x;
//     double angular_vel_change = target_twist_.angular.z - actual_twist_.angular.z;

//     // Clamp velocity changes to acceleration limits
//     linear_vel_change = std::clamp(linear_vel_change, 
//                                  -max_linear_vel_change, 
//                                  max_linear_vel_change);
//     angular_vel_change = std::clamp(angular_vel_change, 
//                                   -max_angular_vel_change, 
//                                   max_angular_vel_change);

//     // Apply limited changes to create acceleration-limited setpoint
//     limited_twist.linear.x = actual_twist_.linear.x + linear_vel_change;
//     limited_twist.angular.z = actual_twist_.angular.z + angular_vel_change;

//     // Apply deadband to limited twist
//     if (abs(limited_twist.linear.x) < velocity_deadband) {
//         limited_twist.linear.x = 0.0;
//         linear_pid_->reset();
//     }
//     if (abs(limited_twist.angular.z) < angular_deadband) {
//         limited_twist.angular.z = 0.0;
//         angular_pid_->reset();
//     }

//     // Command timeout check
//     if ((this->now() - last_cmd_time_).seconds() > 1.0) {
//         limited_twist = geometry_msgs::msg::Twist();
//         linear_pid_->reset();
//         angular_pid_->reset();
//     }

//     // PID control using acceleration-limited setpoint
//     double controlled_linear_vel = linear_pid_->compute(limited_twist.linear.x, actual_twist_.linear.x);
//     double controlled_angular_vel = angular_pid_->compute(limited_twist.angular.z, actual_twist_.angular.z);

//     // Create controlled twist
//     geometry_msgs::msg::Twist controlled_twist;
//     controlled_twist.linear.x = limited_twist.linear.x + controlled_linear_vel;
//     controlled_twist.angular.z = limited_twist.angular.z + controlled_angular_vel;

//     // Debug logging for acceleration limits
//     if (std::abs(linear_vel_change) >= max_linear_vel_change) {
//         RCLCPP_DEBUG(this->get_logger(), 
//             "Linear acceleration limited: target: %.2f, limited: %.2f, actual: %.2f", 
//             target_twist_.linear.x, limited_twist.linear.x, actual_twist_.linear.x);
//     }
//     if (std::abs(angular_vel_change) >= max_angular_vel_change) {
//         RCLCPP_DEBUG(this->get_logger(), 
//             "Angular acceleration limited: target: %.2f, limited: %.2f, actual: %.2f",
//             target_twist_.angular.z, limited_twist.angular.z, actual_twist_.angular.z);
//     }

//     // Rest of your existing code for RPM calculation and publishing
//     controlled_twist.linear.x = std::clamp(controlled_twist.linear.x, 
//                                          -max_linear_velocity_, 
//                                          max_linear_velocity_);
//     controlled_twist.angular.z = std::clamp(controlled_twist.angular.z, 
//                                           -max_angular_velocity_, 
//                                           max_angular_velocity_);

//     // Calculate and publish wheel RPMs based on controlled velocities
//     auto wheel_rpms = calculateWheelRPMs(controlled_twist);
//     limitAndPublishRPMs(wheel_rpms);
// }

void AdvancedTankController::updateCommand() {
    const double velocity_deadband = 0.1;  // 10 cm/s
    const double angular_deadband = 0.01;   // ~0.6 degrees/s

    // If command is very close to zero, explicitly set to zero
    if (abs(target_twist_.linear.x) < velocity_deadband) {
        target_twist_.linear.x = 0.0;
        linear_pid_->reset();  // Reset PID integrator
    }
    if (abs(target_twist_.angular.z) < angular_deadband) {
        target_twist_.angular.z = 0.0;
        angular_pid_->reset();  // Reset PID integrator
    }

    // // Check command timeout (500ms)
    // if ((this->now() - last_cmd_time_).seconds() > 1.0) {
    //     target_twist_ = geometry_msgs::msg::Twist();
    //     linear_pid_->reset();
    //     angular_pid_->reset();
    // }

    // Apply PID control to linear and angular velocities
    double controlled_linear_vel = linear_pid_->compute( target_twist_.linear.x, actual_twist_.linear.x );
    double controlled_angular_vel = angular_pid_->compute( target_twist_.angular.z, actual_twist_.angular.z );

    // Create controlled twist
    geometry_msgs::msg::Twist controlled_twist;
    controlled_twist.linear.x = target_twist_.linear.x + controlled_linear_vel;
    controlled_twist.angular.z = target_twist_.angular.z + controlled_angular_vel;

    RCLCPP_WARN(this->get_logger(), "Controlled Twist - Linear: %.2f, Angular: %.2f", controlled_twist.linear.x, controlled_twist.angular.z);

    controlled_twist.linear.x = std::clamp(controlled_twist.linear.x, -max_linear_velocity_, max_linear_velocity_);
    controlled_twist.angular.z = std::clamp(controlled_twist.angular.z, -max_angular_velocity_, max_angular_velocity_);

    // Calculate and publish wheel RPMs based on controlled velocities
    auto wheel_rpms = calculateWheelRPMs(controlled_twist);
    limitAndPublishRPMs(wheel_rpms);

    RCLCPP_WARN(this->get_logger(), "Target vs Actual - Linear: %.2f/%.2f, Angular: %.2f/%.2f", target_twist_.linear.x, actual_twist_.linear.x,
        target_twist_.angular.z, actual_twist_.angular.z);
}

double AdvancedTankController::velocityToRPM(double velocity, double wheel_diameter) const {
    // Convert linear velocity (m/s) to RPM
    return (velocity * 60.0) / (M_PI * wheel_diameter);
}

WheelState AdvancedTankController::calculateWheelRPMs(const geometry_msgs::msg::Twist& twist) const {
    WheelState result;
    
    // Ref : https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp [code line no: 261]
    // Calculate track velocities (m/s)
    double left_velocity = twist.linear.x - (twist.angular.z * wheel_separation_ / 2.0);
    double right_velocity = twist.linear.x + (twist.angular.z * wheel_separation_ / 2.0);

    
    RCLCPP_DEBUG(this->get_logger(), "Track velocities - Left: %.2f, Right: %.2f", 
                left_velocity, right_velocity);

    // Convert velocities to RPM
    result.rear_left_rpm = velocityToRPM(left_velocity, rear_wheel_diameter_);
    result.rear_right_rpm = velocityToRPM(right_velocity, rear_wheel_diameter_);
    result.front_left_rpm = velocityToRPM(left_velocity, front_wheel_diameter_);
    result.front_right_rpm = velocityToRPM(right_velocity, front_wheel_diameter_);

    RCLCPP_DEBUG(this->get_logger(), "Calculated RPMs - FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
                result.front_left_rpm, result.front_right_rpm, result.rear_left_rpm, result.rear_right_rpm);

    return result;
}


void AdvancedTankController::limitAndPublishRPMs(const WheelState& wheel_rpms) {
    auto msg = std::make_shared<tank_interface::msg::MotorRPMArray>();
    
    front_wheel_left_target_rpm = wheel_rpms.front_left_rpm;
    front_wheel_right_target_rpm = wheel_rpms.front_right_rpm;
    msg->rpm = { wheel_rpms.front_left_rpm, wheel_rpms.front_right_rpm, wheel_rpms.rear_left_rpm, wheel_rpms.rear_right_rpm };
    
    motor_rpm_pub_->publish(*msg);
}


void AdvancedTankController::setupTimers() {
    // Command update timer (20Hz)
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/motor_command_frequency_)),
        std::bind(&AdvancedTankController::updateCommand, this));

    // Odometry publish timer (10Hz)
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/odom_frequency_)),
        std::bind(&AdvancedTankController::publishOdometry, this));
}

void AdvancedTankController::wheelRPMCallback(const tank_interface::msg::MotorRPMArray::SharedPtr msg) {
    current_wheel_state_.front_left_rpm = msg->rpm[0];
    current_wheel_state_.front_right_rpm = msg->rpm[1];
    current_wheel_state_.rear_left_rpm = msg->rpm[2];
    current_wheel_state_.rear_right_rpm = msg->rpm[3];
    updateOdometry();
}

double AdvancedTankController::rpmToVelocity(double rpm, double wheel_diameter) const {
    return (rpm * M_PI * wheel_diameter) / 60.0;
}

void AdvancedTankController::updateOdometry() {
    auto current_time = this->now();
    double dt = (current_time - last_odom_time_).seconds();
    last_odom_time_ = current_time;

    if (dt == 0.0) return;

    // Calculate linear velocities for left and right tracks using front wheels
    double left_front_vel = rpmToVelocity(current_wheel_state_.front_left_rpm, front_wheel_diameter_);
    double right_front_vel = rpmToVelocity(current_wheel_state_.front_right_rpm, front_wheel_diameter_);

    // Calculate robot's linear and angular velocity
    double linear_vel = (right_front_vel + left_front_vel) / 2.0;
    double angular_vel = (right_front_vel - left_front_vel) / wheel_separation_;

    // Update pose
    double delta_theta = angular_vel * dt;
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

    RCLCPP_DEBUG(this->get_logger(), "Current Velocity: %.2f, %.2f , current RPM FL %.2f FR %.2f", linear_vel, angular_vel, current_wheel_state_.front_left_rpm, current_wheel_state_.front_right_rpm);
    RCLCPP_DEBUG(this->get_logger(), "Target Velocity: %.2f, %.2f , Target RPM FL %.2f FR %.2f", target_twist_.linear.x , target_twist_.angular.z, front_wheel_left_target_rpm, front_wheel_right_target_rpm);

    // Update current twist for odometry message
    actual_twist_.linear.x = linear_vel;
    actual_twist_.angular.z = angular_vel;
}

void AdvancedTankController::publishOdometry() {
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
    odom_msg.twist.twist = actual_twist_;

    // Add covariance (example values - adjust based on your robot's characteristics)
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

    // Publish transform
    publishTransform(odom_msg);
}

void AdvancedTankController::publishTransform(const nav_msgs::msg::Odometry& odom_msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_msg.header;
    transform.child_frame_id = odom_msg.child_frame_id;
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = odom_msg.pose.pose.position.z;
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
}

} // namespace tank_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tank_controller::AdvancedTankController>());
    rclcpp::shutdown();
    return 0;
}