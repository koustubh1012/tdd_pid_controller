#include "pid_controller.hpp"
#include <cmath>
#include <iostream>

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
void PIDController::move_robot() {
}

// Compute the velocity based on the PID formula
double PIDController::compute_velocity(double pos) {
    double velocity = 10.0; //placeholder
    return velocity;
}


double PIDController::update_position() {
    double new_position = 1.0;
    return new_position;
}


