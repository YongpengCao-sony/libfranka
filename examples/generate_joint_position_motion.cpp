// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>


#include "yaml-cpp/yaml.h"

#include "examples_common.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  YAML::Node joint_names, waypoints;
  joint_names, waypoints = read_yaml_traj();

  std::cout<<waypoints[0]<<std::endl;
  std::cout<<"the first position value is "<<waypoints[0]["positions"][0]<<std::endl;
  std::vector<double> positionstart = waypoints[0]["positions"].as<std::vector<double>>();
  std::vector<double> positionend = waypoints[waypoints.size() - 1]["positions"].as<std::vector<double>>(); 

  std::vector<double> converted_panda_goal = ur_jntpos_conversion(positionstart);  

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <execution-time>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    double total_period = std::stod(argv[2]);

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // int n = sizeof(q_goal) / sizeof(q_goal[0]);
    // std::vector<double> q_goal_vector(q_goal, q_goal + n);
    int n = 7;
    std::copy(converted_panda_goal.begin(), converted_panda_goal.end(), q_goal.begin());
    std::cout<<"q_goal start :  "<<positionstart[1] << "and " <<positionstart[2]  <<std::endl;
    
    // q_goal = {positionstart.insert(positionstart.begin() + 2, 0)};
    // std::cout<<q_goal[0][1]<<std::endl;

    return -1;
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    robot.control(motion_generator);
    

    std::cout << "Finished moving to initial joint configuration." << std::endl;

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 7> initial_position;
    double time = 0.0;
    robot.control([&initial_position, &time, &total_period](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      double delta_angle = M_PI / 8.0 * (1 - std::cos(2 * M_PI * time/total_period));

      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3] + delta_angle,
                                        initial_position[4] + delta_angle, initial_position[5],
                                        initial_position[6] + delta_angle}};

      if (time >= total_period) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
