#include <arm.h>
#include <stdio.h>

#include <sockpp/udp_socket.h>
#include <pthread.h>

struct args_struct
{
  RobotArm *arm;
  sockpp::udp_socket *behaviour_addr;
  args_struct();
};

void *handStateThread(void *args)
{
  std::cout << "Spawning command reciever thread" << std::endl;

  args_struct *a = (args_struct *)args;
  uint16_t msg, extent;

  while (1)
  {
    auto n = a->behaviour_addr->recv(&msg, sizeof(msg));

    // the last 7 most significant bits out of total 16 are the extent of the reach
    // extent = msg >> 9;

    a->arm->setTargetPosition((float)msg / 100);
  }

  return NULL;
}

int main(int argc, char **argv)
{
  pthread_t handpositionCommand;

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
      std::cerr << "UDP socket bind: " << err << std::endl;
    }

    struct args_struct *args = (args_struct*) malloc(sizeof(struct args_struct));
    args->arm = &arm;
    args->behaviour_addr = &sock;

    // spawn new thread for reading the state of the hand
    pthread_create(
        &handpositionCommand, NULL, &handStateThread, args);

    // start cartesian control loop
    arm.reachAndGrasp();
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
