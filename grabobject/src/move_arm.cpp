// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "common.h"

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <iostream>
#include <unistd.h>

constexpr const char* const SERIAL_PORT = "/dev/ttyUSB0" ;
double position(double start_angle, double end_angle, double time,
                double time_now) {
  // double a_0 = start_angle;
  double a_2 = 3 / (time * time) * (end_angle - start_angle);
  double a_3 = -2 / (time * time * time) * (end_angle - start_angle);

  return (a_2 * (time_now * time_now)) +
         (a_3 * (time_now * time_now * time_now));
}

void connect_to_hand(const std::string& serialport, LibSerial::SerialPort& serial_port)
{
    // Open the hardware serial ports.
    serial_port.Open( serialport ) ;

    // Set the baud rate.
    serial_port.SetBaudRate( LibSerial::BaudRate::BAUD_115200 ) ;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    LibSerial::SerialPort hand_port;
    connect_to_hand("/dev/ttyUSB0", hand_port);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {
        {0, -M_PI_2, 0, -0.9 * M_PI, 0, 0.75 * M_PI, M_PI_2}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!"
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the
    // control loop! Set collision behavior.
    robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 7> initial_position;
    std::array<std::array<double, 7>, 3> movement_angles = {{
        {0, M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2},       // reach
        {0, 0.8 * M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2}, // lift
        q_goal // return to zero position
    }};
    // fastest possible
    // std::array<double, 3> movement_duration = {2.0, 1.0, 2.0};
    std::array<double, 3> movement_duration = {5.0, 1.0, 5.0};

    for (int i = 0; i < 3; i++) {
      double time = 0.0;

      robot.control([&initial_position, &time, &movement_duration,
                     &movement_angles,
                     &i, &hand_port](const franka::RobotState &robot_state,
                         franka::Duration period) -> franka::JointPositions {
        time += period.toSec();

        if (time == 0.0) {
          initial_position = robot_state.q_d;
        }

        double delta_shoulder =
            position(initial_position[1], movement_angles[i][1],
                     movement_duration[i], time);
        double delta_elbow =
            position(initial_position[3], movement_angles[i][3],
                     movement_duration[i], time);
        double delta_wrist =
            position(initial_position[5], movement_angles[i][5],
                     movement_duration[i], time);

        franka::JointPositions output = {
            {initial_position[0], initial_position[1] + delta_shoulder,
             initial_position[2], initial_position[3] + delta_elbow,
             initial_position[4], initial_position[5] + delta_wrist,
             initial_position[6]}};

        if (time >= movement_duration[i]) {
          std::cout << std::endl << "Finished phase " << i << std::endl;

          if (i == 0) {
            // arm has reached the object, grab object
            std::cout << std::endl << "Grabbing Object" << std::endl;
            std::string grab_command = "@AGSM07045++++++*\r" ;
            hand_port.Write(grab_command);

          } else if (i == 1) {
            // arm has lifted the object, release object
            std::cout << std::endl << "Releasing Object" << std::endl;
            std::string release_command = "@AGSM00045++++++*\r" ;
            hand_port.Write(release_command);
          }

          return franka::MotionFinished(output);
        }
        return output;
      });
    }
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
