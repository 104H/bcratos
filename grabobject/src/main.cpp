#include <arm.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  try
  {
    RobotArm arm = RobotArm("/dev/ttyUSB0", "172.16.0.2");

    std::array<double, 7> initial_position;
    std::array<std::array<double, 7>, 3> movement_angles = {{
        {0, -M_PI_2, 0, -0.9 * M_PI, 0, 0.75 * M_PI, M_PI_2}, // initial position
        {0, M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2},       // reach
        {0, 0.8 * M_PI_4, 0, -0.5 * M_PI, 0, M_PI, M_PI_2} // lift
    }};

    // fastest possible
    // std::array<double, 3> movement_duration = {2.0, 1.0, 2.0};
    std::array<double, 3> movement_duration = {5.0, 1.0, 5.0};

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!"
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    for (int i = 0; i < 3; i++)
    {
      double time = 0.0;

      arm.moveArm(movement_angles[i], movement_duration[i]);

        if (i == 0) {
            // arm has reached the object, grab object
            std::cout << std::endl << "Grabbing Object" << std::endl;
            arm.gripObject();

          } else if (i == 1) {
            // arm has lifted the object, release object
            std::cout << std::endl << "Releasing Object" << std::endl;
            arm.releaseObject();
          }

    }
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
