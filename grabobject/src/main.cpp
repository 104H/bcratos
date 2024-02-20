#include <arm.h>
#include <stdio.h>

#include <sockpp/udp_socket.h>
#include "yaml-cpp/yaml.h"

int main(int argc, char **argv)
{
  uint16_t msg;

  YAML::Node config = YAML::LoadFile("config.yaml");

  // raise an error if ip address and port of listener is not defined
  if (!config["behaviourListenerIP"] & !config["behaviourListenerPort"])
  {
    throw std::runtime_error(std::string("IP Address and Port Number not defined for Behaviour Computer Listener."));
  }

  // raise an error if ip address and port of decoder is not defined
  if (!config["decoderListenerIP"] & !config["decoderListenerPort"])
  {
    throw std::runtime_error(std::string("IP Address and Port Number not defined for Decoder Computer."));
  }

  // raise an error if hand port is not defined
  if (!config["handPort"])
  {
    throw std::runtime_error(std::string("Hand port path not defined."));
  }

  try
  {
    RobotArm arm = RobotArm(config["handPort"].as<std::string>());

    sockpp::initialize();
    sockpp::udp_socket sock_position_command, sock_grasp_state;
    sockpp::inet_address addr(config["behaviourListenerIP"].as<std::string>(), config["behaviourListenerPort"].as<int16_t>()); // listener address for behaviour computer

    // speaker socket to hand position
    if (auto err = sock_position_command.bind(sockpp::inet_address(config["decoderListenerIP"].as<std::string>(), config["decoderListenerPort"].as<int16_t>())))
    {
      std::cerr << "UDP socket bind: " << err.error_message() << std::endl;
    }

    while (1)
    {
      auto n = sock_position_command.recv(&msg, sizeof(msg));

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
