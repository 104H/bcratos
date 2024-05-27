#include <arm.h>
#include <stdio.h>

#include <sockpp/udp_socket.h>

int main(int argc, char **argv)
{
  uint16_t msg, extent;

  try
  {
    RobotArm arm = RobotArm("/dev/ttyUSB0", "172.16.0.2");

    std::array<double, 7> initial_position;

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!"
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    sockpp::initialize();
    sockpp::udp_socket sock;

    if (auto err = sock.bind(sockpp::inet_address("localhost", 1400)))
    {
      std::cerr << "Error binding the UDP socket: " << err.error_message() << std::endl;
      return -1;
    }

    arm.reachAndGrasp();

    while (1)
    {
      std::cout << "Listening on socket" << std::endl;
      auto n = sock.recv(&msg, sizeof(msg));
      
      // the last 7 most significant bits out of total 16 are the extent of the reach
      extent = msg >> 9;

      arm.setTargetPosition((float) extent / 100);
    }
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
