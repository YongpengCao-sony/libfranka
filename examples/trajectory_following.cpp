#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example trajectory_following.cpp
 * Move the panda robot to joint positions based on the UR trajectory file.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
              << "<speed-factor>" << std::endl
              << "joint0 to joint6 are joint angles in [rad]." << std::endl
              << "speed-factor must be between zero and one." << std::endl;
    return -1;
  }
  try {

    YAML::Node joint_names, waypoints = read_yaml_traj();
    std::vector<double> positionstart = waypoints[0]["positions"].as<std::vector<double>>();
    std::vector<double> positionend = waypoints[waypoints.size() - 1]["positions"].as<std::vector<double>>(); 
    std::vector<double> converted_panda_goal = ur_jntpos_conversion(positionstart);  
    std::vector<double> converted_panda_goal_end = ur_jntpos_conversion(positionend);  

    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal;
    std::copy(converted_panda_goal.begin(), converted_panda_goal.end(), q_goal.begin());

    double speed_factor = std::stod(argv[2]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    MotionGenerator motion_generator(speed_factor, q_goal);
    
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "init Motion finished" << std::endl;
    std::copy(converted_panda_goal_end.begin(), converted_panda_goal_end.end(), q_goal.begin());
    MotionGenerator motion_generator2(speed_factor, q_goal);
    robot.control(motion_generator);

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
