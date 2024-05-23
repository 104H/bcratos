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
      {0, 0.8 * M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2},  // lift [initial position]
  }};

  const std::array<double, 3> movement_duration = {1.0, 1.0};

public:
  RobotArm(const std::string hand_serial_port, const std::string arm_host_name);
  void setDefaultBehavior();
  void moveToStart();
  void gripObject();
  void releaseObject();
  double position(double start_angle, double end_angle, double time,
                  double time_now);
  void reachAndGrab(float const extent);
  const std::array<double, 7> scaleAngles(std::array<double, 7> start_angles, std::array<double, 7> end_angles, const float extent);
  const double computeExtent (const std::array<double, 7> angles);
};
