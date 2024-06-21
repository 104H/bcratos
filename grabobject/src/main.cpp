#include <arm.h>
#include <stdio.h>
#include <pthread.h>

#include <sockpp/udp_socket.h>
#include <pthread.h>

#include <yaml-cpp/yaml.h>

#include <boost/log/trivial.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

struct args_struct
{
  RobotArm *arm;
  sockpp::inet_address *behaviour_addr;
  sockpp::inet_address *stimulator_addr;
  sockpp::udp_socket *sock_position_command;
  args_struct();
};

void *handStateThread(void *args)
{
  std::cout << "Spawning command receiver thread" << std::endl;

  args_struct *a = (args_struct *)args;
  sockpp::udp_socket sock_behaviour, sock_stimulator;
  uint16_t msg, handCommand, armCommand;

  while (1)
  {
    // recieve command message from behaviour pc
    auto n = a->sock_position_command->recv(&msg, sizeof(msg));

    // the 8 most significant bits out of total 16 are the extent of the arm's reach
    armCommand = msg >> 8;

    // the 8 least significant bits out of total 16 are the extent of the hand's closure
    handCommand = (uint8_t)msg;

    BOOST_LOG_TRIVIAL(debug) << "Received message: " << msg;
    BOOST_LOG_TRIVIAL(debug) << "Command to hand: " << handCommand;
    BOOST_LOG_TRIVIAL(debug) << "Command to arm: " << armCommand;

    // close the hand according to the command
    a->arm->gripObject(handCommand);

    // position the arm according to the command
    a->arm->setTargetPosition((float)armCommand / 180);

    // update the state variable for the hand
    a->arm->updateState();
    std::string state = a->arm->getGrasped() ? "1" : "0";

    if (auto err = sock_behaviour.send_to(state, *(a->behaviour_addr)))
    {
      BOOST_LOG_TRIVIAL(debug) << "Sending hand state to behaviour computer: " << err.error_message();
    }

    if (auto err = sock_stimulator.send_to(state, *(a->stimulator_addr)))
    {
      BOOST_LOG_TRIVIAL(debug) << "Sending hand state to stimulator computer: " << err.error_message();
    }
  }

  return NULL;
}

int main(int argc, char **argv)
{
  boost::log::add_common_attributes();
  boost::log::add_file_log(
      boost::log::keywords::file_name = "logs/log_%N.log", boost::log::keywords::rotation_size = 10 * 1024 * 1024, boost::log::keywords::format = "[%TimeStamp%]: %Message%", boost::log::keywords::auto_flush = true);
  boost::log::core::get()->set_filter(
      boost::log::trivial::severity >= boost::log::trivial::trace);
  BOOST_LOG_TRIVIAL(info) << "Start";

  uint16_t msg;
  pthread_t handpositionCommand;

  YAML::Node config = YAML::LoadFile("config.yaml");

  // raise an error if ip address and port of stimulator is not defined
  if (!config["stimulatorListenerIP"] & !config["stimulatorListenerPort"])
  {
    std::string err = "IP Address and Port Number not defined for Stimulator Computer Listener.";
    throw std::runtime_error(std::string(err));
    BOOST_LOG_TRIVIAL(fatal) << err;
  }

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
    RobotArm arm = RobotArm(config["handPort"].as<std::string>(), config["armIP"].as<std::string>());

    sockpp::initialize();
    sockpp::udp_socket sock_position_command, sock_behaviour, sock_stimulator;
    // listener address for behaviour computer
    sockpp::inet_address behaviour_addr(config["behaviourListenerIP"].as<std::string>(), config["behaviourListenerPort"].as<int16_t>());
    // listener address for stimulator computer
    sockpp::inet_address stimulator_addr(config["stimulatorListenerIP"].as<std::string>(), config["stimulatorListenerPort"].as<int16_t>());

    // speaker socket to hand position
    auto err = sock_position_command.bind(sockpp::inet_address(config["decoderListenerIP"].as<std::string>(), config["decoderListenerPort"].as<int16_t>()));
    BOOST_LOG_TRIVIAL(debug) << "UDP socket bind: " << err.error_message();

    struct args_struct *args = (args_struct *)malloc(sizeof(struct args_struct));
    args->arm = &arm;
    args->behaviour_addr = &behaviour_addr;
    args->stimulator_addr = &stimulator_addr;
    args->sock_position_command = &sock_position_command;

    // spawn new thread for reading the state of the hand
    pthread_create(
        &handpositionCommand, NULL, &handStateThread, args);

    // start cartesian control loop
    arm.reachAndGrasp();

    pthread_join(handpositionCommand, NULL);
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    BOOST_LOG_TRIVIAL(fatal) << e.what();
    return -1;
  }
}