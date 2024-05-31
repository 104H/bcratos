#pragma once

#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <regex>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <eigen3/Eigen/Dense>
#include <boost/log/trivial.hpp>

class RobotArm
{
private:
  // mia hand port
  LibSerial::SerialPort hand;
  std::string full_readout;
  bool grasped; // is the hand in a grasped state

  // franka emika arm
  franka::Robot arm;

  // define the target position
  franka::RobotState initial_state;
  Eigen::Affine3d initial_transform;
  Eigen::Vector3d position_d;
  Eigen::Quaterniond orientation_d;
  Eigen::Vector3d position_start;
  Eigen::Vector3d position_final = {0.84, -0.02, 0.18};

  const std::array<std::array<double, 7>, 3> movement_angles = {{
      {0, M_PI_4, 0, -0.5 * M_PI, 0, 0.9 * M_PI, M_PI_2},        // reach
      {0, 0.8 * M_PI_4, 0, -0.45 * M_PI, 0, 0.9 * M_PI, M_PI_2}, // lift [initial position]
  }};

  const std::array<double, 3> movement_duration = {1.0, 1.0};

  /**
   * Reads out the current position of the hand. For a detailed understanding of hand position parameters, see mia hand docs
   * https://www.prensilia.com/wp-content/uploads/2021/06/210529_Mia_UserManual_web.pdf
   *
   * bool read_success : True if a read out was success full, False if not
   * uint8_t thumb : position of the thumb from 0 to 255 where 0 is fully extended and 255 is fully flexed
   * uint8_t mrl : position of the thumb from 0 to 255 where 0 is fully extended and 255 is fully flexed
   * uint8_t index : position of the thumb from 0 to 255 where 0 is fully extended and 255 is fully flexed
   */
  void readoutPosition(bool &read_success, uint8_t &thumb, uint8_t &mrl, uint8_t &index);

  /**
   * Takes the thumb, middle ring and little, and index finger position to determine if they are in a grasped state
   */
  void isGraspComplete(const uint8_t &thumb, const uint8_t &mrl, const uint8_t &index);

  void setPosition_d(const float target);
  const float getPosition_d();

public:
  /**
   * Constructor for serial port of the hand
   *
   * string hand_serial_port : file path to the serial port as defined in a linux system. e.g "/dev/ttyUSB0"
   */
  RobotArm(const std::string hand_serial_port, const std::string arm_host_name);

  /**
   * Getter function for variable grasped
   */
  bool getGrasped();

  /**
   * Move the hand to the starting position of extended fingers and thumb, an open grip.
   */
  void moveToStart();

  /**
   * Close the grip of the hand on a scale of 0 to 100, where 0 is a fully open grip and 100 is a grip closed enough to grip the object in experiment
   *
   * uint8_t extent : The extent of the grip from 0 to 100
   */
  void gripObject(const uint8_t extent);

  /**
   * Open the hand to a fully open grip.
   */
  void releaseObject();

  double position(double start_angle, double end_angle, double time,
                  double time_now);
  void setDefaultBehavior();
  void reachAndGrasp();
  const float determinePositionFromCommand(const float command);
  void setTargetPosition(const float command);

  /**
   * Readout position of the hand, determine its state and update the variable grasped accordingly
   */
  void updateState();
};
