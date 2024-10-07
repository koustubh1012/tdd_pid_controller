#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <vector>

//Test to check if the robot does not move if it is already at target 
TEST(PIDControllerTest, NoMovementTest_1){
    // Create object of PIDController class
    PIDController controller(0.0, 0.0, 0.1, 0.1);
    std::vector<double> robot_state;
    //set the target for the robot
    controller.set_target(0, 0);
    //tune the PID parameters
    controller.tune_PID(1, 0, 0.1);
    // move the robot and return the final x and y coordinates
    robot_state = controller.move_robot();

    // Test if the x and y positions are correct
    EXPECT_NEAR(robot_state[0], 0, 0.1);
    EXPECT_NEAR(robot_state[1], 0, 0.1);
}
    // Create object of PIDController class
//Test to check if the robot does not move if it is already at target 
TEST(PIDControllerTest, NoMovementTest_2){

    PIDController controller(3.0, 4.0, 0.1, 0.1);
    std::vector<double> robot_state;
    //set the target for the robot
    controller.set_target(3, 4);
    //tune the PID parameters
    controller.tune_PID(1, 0, 0.1);
    // move the robot and return the final x and y coordinates
    robot_state = controller.move_robot();

    // Test if the x and y positions are correct
    EXPECT_NEAR(robot_state[0], 3, 0.1);
    EXPECT_NEAR(robot_state[1], 4, 0.1);
}

// Test to check if the PID controller can move the robot to the target position
TEST(PIDControllerTest, MovementTest_1){
    // Create object of PIDController class
    PIDController controller(0.0, 0.0, 0.1, 0.1);
    std::vector<double> robot_state;
    //set the target for the robot
    controller.set_target(5, 6);
    //tune the PID parameters
    controller.tune_PID(1, 0, 0.1);
    // move the robot and return the final x and y coordinates
    robot_state = controller.move_robot();

    // Test if the x and y positions are correct
    EXPECT_NEAR(robot_state[0], 5, 0.1);
    EXPECT_NEAR(robot_state[1], 6, 0.1);
}

// Test to check if the PID controller can move the robot to the target position
TEST(PIDControllerTest, MovementTest_2){
    // Create object of PIDController class
    PIDController controller(2.0, 3.0, 0.1, 0.1);
    std::vector<double> robot_state;
    //set the target for the robot
    controller.set_target(20, 30);
    //tune the PID parameters
    controller.tune_PID(1, 0, 0.1);
    // move the robot and return the final x and y coordinates
    robot_state = controller.move_robot();

    // Test if the x and y positions are correct
    EXPECT_NEAR(robot_state[0], 20, 0.1);
    EXPECT_NEAR(robot_state[1], 30, 0.1);
}
