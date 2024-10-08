#include "pid_controller.hpp"
#include <iostream>
#include <cmath>
#include <vector>

int main(){

    // Initiallizing the class object "controller"
    PIDController controller(0,0,0.2,1);

    // Setting the target position to 50, 50
    controller.set_target(50, 50);

    // Initializing the gains
    controller.tune_PID(0.01, 0, 0);

    // Calling the function to run the controller and check the target if reached
    std::vector<double> new_position = controller.move_robot();

    std::cout<< " Target Reached" << " " << new_position[0] <<", "<< new_position[1] << std::endl;


    return 0;
}