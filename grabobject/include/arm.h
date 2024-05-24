#pragma once

#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <eigen3/Eigen/Dense>

class RobotArm
{
private:
  // mia hand port
  LibSerial::SerialPort hand;

  // franka emika arm
  franka::Robot arm;

  // define the target position
  franka::RobotState initial_state;
  Eigen::Affine3d initial_transform;
  Eigen::Vector3d position_d;
  Eigen::Quaterniond orientation_d;
  Eigen::Vector3d position_start;
  Eigen::Vector3d position_final = {0.84, -0.02, 0.35};

  const std::array<std::array<double, 7>, 3> movement_angles = {{
      {0, M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2},       // reach
      {0, 0.8 * M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2}, // lift [initial position]
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
  const double computeExtent(const std::array<double, 7> angles);
  const float determinePositionFromCommand(const int command);
  void setPosition_d(const float target);
  void setTargetPosition(const int command);
};
