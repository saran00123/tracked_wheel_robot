#include "tank_drive_controller/advanced_tank_drive_controller.hpp"

namespace tank_controller {

AdvancedTankController::AdvancedTankController() 
    : Node("advanced_tank_controller") {
    
    initializeParameters();
    setupSubscribersAndPublishers();
    setupTimers();

    last_cmd_time_ = this->now();
    wheel_diameter_ratio_ = front_wheel_diameter_ / rear_wheel_diameter_;

    front_wheel_left_target_rpm = 0.0;
    front_wheel_right_target_rpm = 0.0;

    RCLCPP_INFO(this->get_logger(), "Advanced Tank Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Front wheel diameter: %.3f m", front_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Rear wheel diameter: %.3f m", rear_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "Motor Command Frequency: %d Hz", motor_command_frequency_);
    RCLCPP_INFO(this->get_logger(), "Use PID: %s", use_pid ? "true" : "false");
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
    this->declare_parameter("motor_command_frequency", 50);   // Hz

    // PID parameters
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
    motor_command_frequency_ = this->get_parameter("motor_command_frequency").as_int();
    
    linear_pid_kp_ = this->get_parameter("linear_pid_kp").as_double();
    linear_pid_ki_ = this->get_parameter("linear_pid_ki").as_double();
    linear_pid_kd_ = this->get_parameter("linear_pid_kd").as_double();
    angular_pid_kp_ = this->get_parameter("angular_pid_kp").as_double();
    angular_pid_ki_ = this->get_parameter("angular_pid_ki").as_double();
    angular_pid_kd_ = this->get_parameter("angular_pid_kd").as_double();
    pid_windup_limit_ = this->get_parameter("pid_windup_limit").as_double();
    
    if(use_pid) {
        RCLCPP_INFO(this->get_logger(), "Using PID controllers");
        // Initialize PID controllers
        linear_pid_ = std::make_unique<PIDController>(
            linear_pid_kp_, linear_pid_ki_, linear_pid_kd_,
            -max_linear_velocity_, max_linear_velocity_, pid_windup_limit_);
            
        angular_pid_ = std::make_unique<PIDController>(
            angular_pid_kp_, angular_pid_ki_, angular_pid_kd_,
            -max_angular_velocity_, max_angular_velocity_, pid_windup_limit_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Not using PID controllers");
    }

}

void AdvancedTankController::setupSubscribersAndPublishers() {
    // Setting up subscribers and publishers

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, 
        std::bind(&AdvancedTankController::cmdVelCallback, this, std::placeholders::_1));

    actual_speed_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/actual_tank_speed", 30, 
        std::bind(&AdvancedTankController::actualSpeedCallback, this, std::placeholders::_1));

    // wheel_rpm_sub_ = this->create_subscription<tank_interface::msg::MotorRPMArray>(
    //     "/actual_wheel_speed", 30, 
    //     std::bind(&AdvancedTankController::wheelRPMCallback, this, std::placeholders::_1));


    motor_rpm_pub_ = this->create_publisher<tank_interface::msg::MotorRPMArray>("motor_rpms", 10);
}

void AdvancedTankController::actualSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual twist from the received message
    actual_twist_.linear.x = msg->linear.x;   // Forward velocity
    actual_twist_.angular.z = msg->angular.z; // Angular velocity
    
    RCLCPP_DEBUG(this->get_logger(), "Actual speeds - Linear: %.2f m/s, Angular: %.2f rad/s", actual_twist_.linear.x, actual_twist_.angular.z);
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


void AdvancedTankController::updateCommand() {
    const double velocity_deadband = 0.1;  // 10 cm/s
    const double angular_deadband = 0.01;   // ~0.6 degrees/s

    // If command is very close to zero, explicitly set to zero
    if (abs(target_twist_.linear.x) < velocity_deadband) {
        target_twist_.linear.x = 0.0;
            if(use_pid) { linear_pid_->reset(); }  // Reset PID integrator
    }
    if (abs(target_twist_.angular.z) < angular_deadband) {
        target_twist_.angular.z = 0.0;
            if(use_pid) { angular_pid_->reset(); }  // Reset PID integrator
    }

    // Check command timeout (500ms)
    // if ((this->now() - last_cmd_time_).seconds() > 1.0) {
    //     target_twist_ = geometry_msgs::msg::Twist();
    //     linear_pid_->reset();
    //     angular_pid_->reset();
    // }
    geometry_msgs::msg::Twist controlled_twist;
    if(use_pid) {
        // Apply PID control to linear and angular velocities
        double controlled_linear_vel = linear_pid_->compute( target_twist_.linear.x, actual_twist_.linear.x );
        double controlled_angular_vel = angular_pid_->compute( target_twist_.angular.z, actual_twist_.angular.z );

        controlled_twist.linear.x = target_twist_.linear.x + controlled_linear_vel;
        controlled_twist.angular.z = target_twist_.angular.z + controlled_angular_vel;
    }
    else{
        controlled_twist.linear.x = target_twist_.linear.x;
        controlled_twist.angular.z = target_twist_.angular.z;
    }
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
    // Command update timer
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/motor_command_frequency_)),
        std::bind(&AdvancedTankController::updateCommand, this));
}

// void AdvancedTankController::wheelRPMCallback(const tank_interface::msg::MotorRPMArray::SharedPtr msg) {
//     current_wheel_state_.front_left_rpm = msg->rpm[0];
//     current_wheel_state_.front_right_rpm = msg->rpm[1];
//     current_wheel_state_.rear_left_rpm = msg->rpm[2];
//     current_wheel_state_.rear_right_rpm = msg->rpm[3];
// }

double AdvancedTankController::rpmToVelocity(double rpm, double wheel_diameter) const {
    return (rpm * M_PI * wheel_diameter) / 60.0;
}

} // namespace tank_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tank_controller::AdvancedTankController>());
    rclcpp::shutdown();
    return 0;
}