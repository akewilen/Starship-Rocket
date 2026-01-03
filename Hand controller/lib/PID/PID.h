#ifndef PID_H
#define PID_H

class PID {
private:
    double kp, ki, kd;           // PID gains
    double output_min, output_max; // Output clamping limits
    double integral;             // Integral term accumulator
    double prev_error;           // Previous error for derivative calculation
    bool first_run;              // Flag for first execution
    
public:
    // Constructor
    PID(double kp, double ki, double kd, double output_min, double output_max);
    
    // Main PID computation function with fixed time step
    double compute(double error, double dt);
    
    // Reset PID state (useful for mode changes)
    void reset();
    
    // Setters for tuning
    void setGains(double kp, double ki, double kd);
    void setOutputLimits(double min_output, double max_output);
    
    // Getters for monitoring
    double getIntegral() const { return integral; }
    double getLastError() const { return prev_error; }
};

#endif
