#include <arm.h>

RobotArm::RobotArm(const std::string hand_serial_port)
{
  hand.Open(hand_serial_port);
  // Set the baud rate.
  hand.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

  // move to a start position of a fully open grip
  moveToStart();

  // turn on streaming for finger positions
  hand.Write("@ADP1-----------*\r");

  // set the grasped variable to false because hand is open
  grasped = false;
}

bool RobotArm::getGrasped()
{
  return grasped;
}

void RobotArm::gripObject(const uint8_t extent)
{
  // scale the extent in the range from 0 to 75
  const int8_t scaled_extent = extent * 0.75;

  // convert to string
  std::string ext = std::to_string(scaled_extent);

  // write to the hand serial port
  hand.Write("@AGSM0" + ext + "45++++++*\r");
}

void RobotArm::readoutPosition(bool &read_success, uint8_t &thumb, uint8_t &mrl, uint8_t &index)
{
  // regular expression for detecting 5 consecutive digits occurring thrice for the thumb, mrl and index positions respectively
  std::regex rgx("enc : [\\+\\-](\\d{5}) ; [\\+\\-](\\d{5}) ; [\\+\\-](\\d{5}) ; [\\+\\-]\\d{5}\\n", std::regex_constants::ECMAScript);
  std::smatch sm;

  // flush the input buffer to get the latest readout
  hand.FlushInputBuffer();

  // readout from the hand
  std::string new_readout;
  hand.Read(new_readout, 40, 60);
  
  // append to has been read out
  full_readout.append(new_readout);

  // search readout with regex
  std::regex_search(full_readout, sm, rgx);

  // if regex detected in string
  if (sm.size() > 0)
  {
    // extract thumb, index and mrl motor positions
    thumb = stoi(sm.str(1));
    mrl = stoi(sm.str(2));
    index = stoi(sm.str(3));

    // std::cout << static_cast<int16_t>(thumb) << std::endl << static_cast<int16_t>(mrl) << std::endl << static_cast<int16_t>(index) << std::endl << std::endl;

    // clear out the string
    full_readout = "";

    read_success = true;
    return;
  }

  read_success = false;
}

void RobotArm::releaseObject() { hand.Write("@AGSM00045++++++*\r"); }

void RobotArm::moveToStart()
{
  releaseObject();
}

void RobotArm::isGraspComplete(const uint8_t &thumb, const uint8_t &mrl, const uint8_t &index)
{
  // grasp is complete if the motor positions are above 150
  uint16_t threshold = 100;
  grasped = (thumb > threshold) & (mrl > threshold) & (index > threshold);
}

void RobotArm::updateState()
{
  bool read_success;
  uint8_t thumb, mrl, index;
  readoutPosition(read_success, thumb, mrl, index);

  // if the readout was successful, update the state of `grasped` according to the new readout
  if (read_success)
  {
    isGraspComplete(thumb, mrl, index);
  }
}