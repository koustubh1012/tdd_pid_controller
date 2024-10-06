#include "pid_controller.hpp"
#include <cmath>
#include <iostream>

// Constructor initializing the member variables
PIDController::PIDController(double curr_x, double curr_y, double d_t, double dist, double thresh, 
         double vel_x, double vel_y, 
         double curr_err_x, double prev_err_x, double acc_err_x, 
         double curr_err_y, double prev_err_y, double acc_err_y) {

    // Initialize member variables with constructor parameters
    current_x = curr_x;
    current_y = curr_y;
    delta_t = d_t;
    distance = dist;
    threshold = thresh;
    calculated_velocity_x = vel_x;
    calculated_velocity_y = vel_y;
    current_error_x = curr_err_x;
    previous_error_x = prev_err_x;
    accumulated_error_x = acc_err_x;
    current_error_y = curr_err_y;
    previous_error_y = prev_err_y;
    accumulated_error_y = acc_err_y;
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
