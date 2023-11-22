#include <arm.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  try
  {
    RobotArm arm = RobotArm("/dev/ttyUSB0", "172.16.0.2");

    std::array<double, 7> initial_position;

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!"
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    arm.reachAndGrab(.7);
    arm.reachAndGrab(.1);
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
