#include <arm.h>
#include <stdio.h>

#include <sockpp/udp_socket.h>
#include <yaml-cpp/yaml.h>

#include <boost/log/trivial.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

int main(int argc, char **argv)
{
  boost::log::add_common_attributes();
  boost::log::add_file_log(
      boost::log::keywords::file_name = "log_%N.log", boost::log::keywords::rotation_size = 10 * 1024 * 1024, boost::log::keywords::format = "[%TimeStamp%]: %Message%", boost::log::keywords::auto_flush = true);
  boost::log::core::get()->set_filter(
      boost::log::trivial::severity >= boost::log::trivial::trace);
  BOOST_LOG_TRIVIAL(info) << "Start";

  uint16_t msg;

  YAML::Node config = YAML::LoadFile("config.yaml");

  // raise an error if ip address and port of listener is not defined
  if (!config["behaviourListenerIP"] & !config["behaviourListenerPort"])
  {
    std::string err = "IP Address and Port Number not defined for Behaviour Computer Listener.";
    throw std::runtime_error(std::string(err));
    BOOST_LOG_TRIVIAL(fatal) << err;
  }

  // raise an error if ip address and port of decoder is not defined
  if (!config["decoderListenerIP"] & !config["decoderListenerPort"])
  {
    std::string err = "IP Address and Port Number not defined for Decoder Computer.";
    throw std::runtime_error(std::string(err));
    BOOST_LOG_TRIVIAL(fatal) << err;
  }

  // raise an error if hand port is not defined
  if (!config["handPort"])
  {
    std::string err = "Hand port path not defined.";
    throw std::runtime_error(std::string(err));
    BOOST_LOG_TRIVIAL(fatal) << err;
  }

  try
  {
    RobotArm arm = RobotArm(config["handPort"].as<std::string>());

    sockpp::initialize();
    sockpp::udp_socket sock_position_command, sock_grasp_state;
    // listener address for behaviour computer
    sockpp::inet_address addr(config["behaviourListenerIP"].as<std::string>(), config["behaviourListenerPort"].as<int16_t>());

    // speaker socket to hand position
    if (auto err = sock_position_command.bind(sockpp::inet_address(config["decoderListenerIP"].as<std::string>(), config["decoderListenerPort"].as<int16_t>())))
    {
      BOOST_LOG_TRIVIAL(info) << "UDP socket bind: " << err.error_message();
    }

    while (1)
    {
      auto n = sock_position_command.recv(&msg, sizeof(msg));
      BOOST_LOG_TRIVIAL(debug) << "Recieved message: " << msg;

      arm.gripObject(msg);

      arm.updateState();

      sock_grasp_state.send_to(arm.getGrasped() ? "1" : "0", addr);
    }
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    BOOST_LOG_TRIVIAL(fatal) << e.what();
    return -1;
  }
}
