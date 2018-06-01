// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <iterator>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

int main(int argc, char** argv) {
  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }
  try {
    franka::Robot robot("172.16.0.2");
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    // read
    // size_t count = 0;
    // robot.read([&count](const franka::RobotState& robot_state) {
    //   // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
    //   // should not be done in a control loop.
    //   std::cout << robot_state.tau_J.data() << std::endl;
    //   return count++ < 100;
    // });

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);

    // read robot state
//   franka::RobotState state = robot.readOnce();
//   franka::Model model(robot.loadModel());

   // for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector; frame++) {
   //   // std::cout << model.bodyJacobian(frame, state) << std::endl;
   //   std::cout << model.pose(frame, state) << std::endl;
   // }


    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
//    robot.setCollisionBehavior(
//        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
//        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
//        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
//        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    constexpr double kRadius = 0.3;
    std::array<double, 16> initial_pose;
    double time = 0.0;

    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_d;//desired ee pose
      }

      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = kRadius * std::sin(angle);
      double delta_z = kRadius * (std::cos(angle) - 1);

      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;

//      std::cout << "time: " << time << " , new pose:" << delta_x << " , " << delta_z <<  " \n"
//    		  << new_pose << std::endl;
      std::cout << robot_state.F_x_Cee << " , " << robot_state.m_ee << " , " << robot_state.m_load
    		  << "  ,\n joint torques"   << robot_state.tau_J << " , cartesian collision: " << robot_state.cartesian_collision << std::endl;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
