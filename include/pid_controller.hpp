/**
 * @file pid_controller.hpp
 * @author FNU Koustubh (koustubh@umd.edu)
 * @brief PIDController class for robots
 * @version 0.1
 * @date 2024-10-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include <iostream>
#include <vector>

/**
 * @brief PIDController class to implement PID controller
 * @class PIDController
 */
class PIDController {
   private:
   /**
    * @brief The current X position of the robot
    *
    */
   double current_x;
   /**
    * @brief The current y position of the robot
    * 
    */
   double current_y;
   /**
    * @brief The time frequency iof the PID controller
    * 
    */
   double delta_t;
   /**
    * @brief Distance to the target
    * 
    */
   double distance;
   /**
    * @brief Allowed error from target
    * 
    */
   double threshold;
   /**
    * @brief Target X coordinate
    * 
    */
   double target_x;
   /**
    * @brief Target Y coordinate
    * 
    */
   double target_y;
   /**
    * @brief Proportionl gain
    * 
    */
   double Kp;
   /**
    * @brief Integral gain
    * 
    */
   double Ki;
   /**
    * @brief Differential gain
    * 
    */
   double Kd;
   /**
    * @brief calculated x axis velocity
    * 
    */
   double calculated_velocity_x;
   /**
    * @brief calculated x axis velocity
    * 
    */
   double calculated_velocity_y;
   /**
    * @brief Current error from target x
    * 
    */
   double current_error_x;
   /**
    * @brief Previous error from target x
    * 
    */
   double previous_error_x;
   /**
    * @brief Accumulated error from target x
    * 
    */
   double accumulated_error_x;
   /**
    * @brief Current error from target y
    * 
    */
   double current_error_y;
   /**
    * @brief Previous error from target y
    * 
    */
   double previous_error_y;
   /**
    * @brief Accumulated error from target y
    * 
    */
   double accumulated_error_y;

   /**
    * @brief Function to update coordinates in 1 dimension
    * 
    * @return double 
    */
   double update_position();

   public:

   /**
    * @brief Construct a new PIDController object
    * 
    * @param robot_x The robot start x coordinate
    * @param robot_y The robot start y coordinate
    * @param d_t  The time frequency of the controller
    * @param thresh  The allowed threshold error from target
    */
   PIDController(double robot_x, double robot_y, double d_t, double thresh);

   /**
    * @brief Set the target of the robot
    * 
    * @param target_x Target X coordinate
    * @param target_y target Y coordinate
    */
   void set_target(double target_x, double target_y);

   /**
    * @brief Tune the PID gains of the controller
    * 
    * @param K_p Proportional gain 
    * @param K_i Integral gain
    * @param K_d Differential gain
    */
   void tune_PID(double K_p, double K_i, double K_d);

   /**
    * @brief Function to implement a closed loop controller
    * 
    * @return std::vector<double> Final X and Y coordinates of the robot
    */
   std::vector<double> move_robot();

   /**
    * @brief Computes the velocity in 1 direction using PID
    * 
    * @param pos The current position of robot in 1 dimension
    * @return double Thee calculate velocity of the robot
    */
   double compute_velocity(double pos);
};