#pragma once
#include <iostream>

/**
 * @brief PIDController Class Constructor
 * 
 */
class PIDController {
   private:
   
   double current_x;
   double current_y;
   double delta_t;
   double distance;
   double threshold;
   double target_x;
   double target_y;
   double Kp;
   double Ki;
   double Kd;
   double calculated_velocity_x;
   double calculated_velocity_y;
   double current_error_x;
   double previous_error_x;
   double accumulated_error_x;
   double current_error_y;
   double previous_error_y;
   double accumulated_error_y;

   public:
   
PIDController(double curr_x, double curr_y, double d_t, double dist, double thresh, 
         double vel_x, double vel_y, double curr_err_x, double prev_err_x,
         double acc_err_x, double curr_err_y, double prev_err_y, 
         double acc_err_y);
   void set_target(double targ_x, double targ_y);
   void tune_PID(double K_p, double K_i, double K_d);
   void move_robot();
   double compute_velocity(double pos);
   double update_position();
};