#pragma once

#include <iostream>
#include <regex>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

class RobotArm
{
private:
  // mia hand port
  LibSerial::SerialPort hand;
  std::string full_readout;
  bool grasped; // is the hand in a grasped state
  void readoutPosition(bool &read_success, uint8_t &thumb, uint8_t &mrl, uint8_t &index);

  /**
   * Takes the thumb, middle ring and little, and index finger position to determine if they are in a grasped state
  */
  void isGraspComplete(const uint8_t &thumb, const uint8_t &mrl, const uint8_t &index);

public:
  RobotArm(const std::string hand_serial_port);

  /**
   * Getter function for variable grasped
  */
  bool getGrasped();
  void moveToStart();
  void gripObject(const uint8_t extent);
  void releaseObject();

  /**
   * Readout position of the hand, determine its state and update the variable grasped accordingly
  */
  void updateState();
};
