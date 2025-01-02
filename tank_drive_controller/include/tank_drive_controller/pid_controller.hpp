// pid_controller.hpp
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>
#include <algorithm> // For std::clamp

class PIDController {
public:
    PIDController(double kp, double ki, double kd, 
                  double output_min, double output_max, double windup_limit, 
                  double initial_integral = 0.0, double initial_prev_error = 0.0)
        : kp_(kp), ki_(ki), kd_(kd)
        , output_min_(output_min), output_max_(output_max)
        , windup_limit_(windup_limit)
        , integral_(initial_integral), prev_error_(initial_prev_error)
        , last_time_(std::chrono::steady_clock::now()) {}

    double compute(double setpoint, double measured_value) {
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time_).count();
        last_time_ = current_time;

        // Enforce a minimum time step to avoid instability
        if (dt < 1e-6) return 0.0;

        // Calculate error
        double error = setpoint - measured_value;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term with anti-windup
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -windup_limit_, windup_limit_);
        double i_term = ki_ * integral_;

        // Derivative term (considering noise sensitivity)
        double d_term = kd_ * (error - prev_error_) / dt;
        prev_error_ = error;

        // Calculate and clamp total output
        double output = p_term + i_term + d_term;
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

    void setOutputLimits(double output_min, double output_max) {
        output_min_ = output_min;
        output_max_ = output_max;
    }

    void setWindupLimit(double windup_limit) {
        windup_limit_ = windup_limit;
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