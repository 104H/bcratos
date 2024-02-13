#include <arm.h>
#include <stdio.h>

#include <pthread.h>

#include <sockpp/udp_socket.h>

int main(int argc, char **argv)
{
  uint16_t msg;
  pthread_t ptid;

  try
  {
    RobotArm arm = RobotArm("/dev/ttyUSB0");

    sockpp::initialize();
    sockpp::udp_socket sock_position_command, sock_grasp_state;
    sockpp::inet_address addr("192.168.137.20", 1000); // listener address for behaviour computer

    // speaker socket to hand position
    if (auto err = sock_position_command.bind(sockpp::inet_address("192.168.137.40", 4700)))
    {
      std::cerr << "UDP socket bind: " << err.error_message() << std::endl;
    }

    while (1)
    {
      auto n = sock_position_command.recv(&msg, sizeof(msg), MSG_DONTWAIT);

      arm.gripObject(msg);

      // the last 7 most significant bits out of total 16 are the extent of the reach
      msg = msg >> 9;

      arm.updateState();
      
      sock_grasp_state.send_to(arm.getGrasped() ? "1" : "0", addr);
    }
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
