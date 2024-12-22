// pid_controller.hpp
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double output_min, double output_max, double windup_limit)
        : kp_(kp), ki_(ki), kd_(kd)
        , output_min_(output_min), output_max_(output_max)
        , windup_limit_(windup_limit)
        , integral_(0.0), prev_error_(0.0)
        , last_time_(std::chrono::steady_clock::now()) {}

    double compute(double setpoint, double measured_value) {
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time_).count();
        last_time_ = current_time;

        if (dt <= 0.0) return 0.0;

        // Calculate error
        double error = setpoint - measured_value;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term with anti-windup
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -windup_limit_, windup_limit_);
        double i_term = ki_ * integral_;

        // Derivative term (on measurement to avoid derivative kick)
        double d_term = kd_ * (error - prev_error_) / dt;
        prev_error_ = error;

        // Calculate total output
        double output = p_term + i_term + d_term;

        // Clamp output to limits
        return std::clamp(output, output_min_, output_max_);
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        last_time_ = std::chrono::steady_clock::now();
    }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double windup_limit_;
    double integral_;
    double prev_error_;
    std::chrono::steady_clock::time_point last_time_;
};

#endif // PID_CONTROLLER_HPP