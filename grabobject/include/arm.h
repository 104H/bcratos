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

public:
  /**
   * Constructor for serial port of the hand
   *
   * string hand_serial_port : file path to the serial port as defined in a linux system. e.g "/dev/ttyUSB0"
   */
  RobotArm(const std::string hand_serial_port);

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

  /**
   * Readout position of the hand, determine its state and update the variable grasped accordingly
   */
  void updateState();
};
