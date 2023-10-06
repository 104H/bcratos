#pragma once

#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

class RobotArm
{
private:
  // mia hand port
  LibSerial::SerialPort hand;

  // franka emika arm
  franka::Robot arm;

  const std::array<std::array<double, 7>, 3> movement_angles = {{
      {0, M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2},        // reach
      {0, 0.8 * M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2},  // lift
      {0, -M_PI_2, 0, -0.9 * M_PI, 0, 0.75 * M_PI, M_PI_2} // initial position
  }};

  // fastest possible
  // std::array<double, 3> movement_duration = {2.0, 1.0, 2.0};
  const std::array<double, 3> movement_duration = {5.0, 1.0, 5.0};

public:
  RobotArm(const std::string hand_serial_port, const std::string arm_host_name);
  void setDefaultBehavior();
  void gripObject();
  void releaseObject();
  double position(double start_angle, double end_angle, double time,
                  double time_now);
  void reachAndGrab();
};
