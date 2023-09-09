#pragma once

#include <iostream>

#include <eigen3/Eigen/Core>

#include <franka/exception.h>
#include <franka/robot.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

class RobotArm {
private:
  // mia hand port
  LibSerial::SerialPort hand;

  // franka emika arm
  franka::Robot arm;

public:
  RobotArm(const std::string hand_serial_port, const std::string arm_host_name);
  void setDefaultBehavior();
  void gripObject();
  void releaseObject();
  double position(double start_angle, double end_angle, double time,
                  double time_now);
  void moveArm(const std::array<double, 7> &movement_angles,
               const double &time);
};
