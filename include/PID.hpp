#pragma once
#include <iostream>

/**
 * @brief PID Class Constructor
 * 
 */
class PID {
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
   
   PID(double, double, double, double, double, double, double, double, double, 
   double, double, double, double, double, double, double, double, double);
   void set_target(double, double);
   void tune_PID(double, double, double);
   void move_robot();
   double compute_velocity(double);
   double update_position();
};