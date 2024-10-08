#include "pid_controller.hpp"
#include <cmath>
#include <iostream>
#include <vector>

// Constructor initializing the member variables
PIDController::PIDController(double robot_x, double robot_y, double d_t, double thresh) {

    // Initialize member variables with constructor parameters
    current_x = robot_x;
    current_y = robot_y;
    delta_t = d_t;
    distance = 0;
    threshold = thresh;
    calculated_velocity_x = 0;
    calculated_velocity_y = 0;
    current_error_x = 0;
    previous_error_x = 0;
    accumulated_error_x = 0;
    current_error_y = 0;
    previous_error_y = 0;
    accumulated_error_y = 0;
}

// Set the target position
void PIDController::set_target(double targ_x, double targ_y) {
    target_x = targ_x;
    target_y = targ_y;
}

// Tune the PID controller parameters
void PIDController::tune_PID(double K_p, double K_i, double K_d) {
    Kp = K_p;
    Ki = K_i;
    Kd = K_d;
}

// Move the robot based on calculated velocities (this should implement PID control logic)
std::vector<double> PIDController::move_robot() {
    return std::vector<double> {10.0, 10.0};  // placeholder
}

// Compute the velocity based on the PID formula
void PIDController::compute_velocity() {

    // Calculating the proportional error
    current_error_x = target_x - current_x;
    current_error_y = target_y - current_y;

    // Calculating the integral error
    accumulated_error_x +=  current_error_x * delta_t;
    accumulated_error_y += current_error_y * delta_t;

    // Calculating the derivative error
    double der_errx = (current_error_x - previous_error_x) / delta_t;
    double der_erry = (current_error_y - previous_error_y) / delta_t;

    // Adding all the terms
    calculated_velocity_x = Kp * current_error_x + Ki * accumulated_error_x + Kd * der_errx;
    calculated_velocity_y = Kp * current_error_y + Ki * accumulated_error_y + Kd * der_erry;

    previous_error_x = current_error_x;
    previous_error_y = current_error_y;
}

// update the current x and y position according to calculated velocity
void PIDController::update_position() {
    
    // Updating the current x and y positions
    current_x += calculated_velocity_x*delta_t;
    current_y += calculated_velocity_y*delta_t;
}


