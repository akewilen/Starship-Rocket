#include "PID.h"

PID::PID(double kp, double ki, double kd, double output_min, double output_max) 
    : kp(kp), ki(ki), kd(kd), output_min(output_min), output_max(output_max),
      integral(0.0), prev_error(0.0), first_run(true) {
}

double PID::compute(double error, double dt) {
    // Handle first run (no previous error for derivative)
    if (first_run) {
        prev_error = error;
        first_run = false;
        return 0.0; // Return 0 on first run to avoid derivative spike
    }
    
    // Validate time delta
    if (dt <= 0.0) {
        return 0.0; // Avoid division by zero or negative dt
    }
    
    // Proportional term
    double proportional = kp * error;
    
    // Integral term with anti-windup (conditional integration)
    double unclamped_output = proportional + ki * integral + kd * (error - prev_error) / dt;
    
    // Only integrate if output is not saturated OR if integration would help unsaturate
    bool should_integrate = true;
    if (unclamped_output > output_max) {
        should_integrate = (error < 0.0); // Only integrate if error is negative (trying to reduce output)
    } else if (unclamped_output < output_min) {
        should_integrate = (error > 0.0); // Only integrate if error is positive (trying to increase output)
    }
    
    if (should_integrate) {
        integral += error * dt;
    }
    
    // Derivative term
    double derivative = kd * (error - prev_error) / dt;
    
    // Calculate total output
    double output = proportional + ki * integral + derivative;
    
    // Apply output clamping
    if (output > output_max) {
        output = output_max;
    } else if (output < output_min) {
        output = output_min;
    }
    
    // Store current error for next iteration
    prev_error = error;
    
    return output;
}

void PID::reset() {
    integral = 0.0;
    prev_error = 0.0;
    first_run = true;
}

void PID::setGains(double new_kp, double new_ki, double new_kd) {
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
}

void PID::setOutputLimits(double min_output, double max_output) {
    output_min = min_output;
    output_max = max_output;
    
    // Clamp existing integral if needed
    double max_integral = (output_max - kp * prev_error) / ki;
    double min_integral = (output_min - kp * prev_error) / ki;
    
    if (ki != 0.0) {
        if (integral > max_integral) {
            integral = max_integral;
        } else if (integral < min_integral) {
            integral = min_integral;
        }
    }
}
