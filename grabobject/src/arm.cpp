#include <arm.h>

RobotArm::RobotArm(const std::string hand_serial_port, const std::string arm_host_name) : arm(arm_host_name)
{
  hand.Open(hand_serial_port);
  // Set the baud rate.
  hand.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  hand.FlushIOBuffers();

  setDefaultBehavior();

  moveToStart();

  // define the target position
  initial_state = arm.readOnce();
  initial_transform = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());

  // turn on streaming for finger positions
  hand.Write("@ADA1-----------*\r");

  // set the grasped variable to false because hand is open
  grasped = false;
}

bool RobotArm::getGrasped()
{
  return grasped;
}

void RobotArm::gripObject(const uint8_t extent)
{
  // convert to string
  std::string cmd = "@AGeC0" + std::to_string(extent) + "045+++++*\r";
  BOOST_LOG_TRIVIAL(debug) << "Command to Hand: " << cmd;

  // write to the hand serial port
  hand.Write(cmd);
}

void RobotArm::readoutPosition(bool &read_success, uint16_t &thumb, uint16_t &mrl, uint16_t &index)
{
  // regular expression for detecting 5 consecutive digits occurring thrice for the thumb, mrl and index positions respectively
  std::regex rgx("adc : [\\+\\-]\\d{5} ; [\\+\\-](\\d{5}) ; [\\+\\-]\\d{5} ; [\\+\\-]\\d{5} ; [\\+\\-](\\d{5}) ; [\\+\\-](\\d{5}) ; [\\+\\-]\\d{5} ; [\\+\\-]\\d{5} ; [\\+\\-](\\d{5}) ; [\\+\\-](\\d{5})", std::regex_constants::ECMAScript);
  std::smatch sm;

  // flush the input buffer to get the latest readout
  hand.FlushInputBuffer();

  // readout from the hand
  std::string new_readout;
  hand.Read(new_readout, 100, 0);

  // append to has been read out
  full_readout.append(new_readout);
 
  // search readout with regex
  std::regex_search(full_readout, sm, rgx);

  // if regex detected in string
  if (sm.size() > 0)
  {
    // extract thumb, index and mrl motor positions
    index = stoi(sm.str(1));
    thumb = stoi(sm.str(2));
    mrl = stoi(sm.str(3));

    BOOST_LOG_TRIVIAL(debug) << "Thumb, MRL, Index Analogue (Force) Inputs: " << static_cast<int16_t>(thumb) << " " << static_cast<int16_t>(mrl) << " " << static_cast<int16_t>(index);
    BOOST_LOG_TRIVIAL(debug) << "Motor Voltage : " << static_cast<int16_t>(stoi(sm.str(4)));

    // clear out the string
    full_readout = "";

    read_success = true;
    return;
  }

  read_success = false;
}

void RobotArm::releaseObject() { hand.Write("@AGSM00045++++++*\r"); }

double RobotArm::position(double start_angle, double end_angle, double time,
                          double time_now)
{
  double a_2 = 3 / (time * time) * (end_angle - start_angle);
  double a_3 = -2 / (time * time * time) * (end_angle - start_angle);

  return (a_2 * (time_now * time_now)) +
         (a_3 * (time_now * time_now * time_now));
}

void RobotArm::setDefaultBehavior()
{
  arm.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  arm.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  arm.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  // set collision behavior
  arm.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                           {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                           {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                           {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
}

void RobotArm::moveToStart()
{
  std::array<double, 7> initial_position;
  double time = 0.0;
  auto angles = movement_angles[1];
  auto duration = movement_duration[1];

  arm.control([this, &time, &initial_position, &angles,
               &duration](const franka::RobotState &robot_state,
                          franka::Duration period) -> franka::JointPositions
              {
    time += period.toSec();

    if (time == 0.0) {
      initial_position = robot_state.q_d;
      return franka::JointPositions(initial_position);
    }

    std::array<double, 7> delta;
    for (int i = 0; i < 7; i++) {
      delta[i] = position(initial_position[i], angles[i], duration, time);
    }

    franka::JointPositions output = {
        {initial_position[0] + delta[0], initial_position[1] + delta[1],
         initial_position[2] + delta[2], initial_position[3] + delta[3],
         initial_position[4] + delta[4], initial_position[5] + delta[5],
         initial_position[6] + delta[6]}};

    if (time >= duration) {
      std::cout << "Attained Starting Position" << std::endl;

      return franka::MotionFinished(output);
    }
    return output; });

  releaseObject();
}

void RobotArm::reachAndGrasp()
{
  std::cout << "Spawning control thread" << std::endl;

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  // load robot model
  franka::Model model = arm.loadModel();

  // equilibrium point is the initial position
  position_d = initial_transform.translation();
  position_start = initial_transform.translation();
  orientation_d = initial_transform.linear();

  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
      impedance_control_callback = [&](const franka::RobotState &robot_state,
                                       franka::Duration /*duration*/) -> franka::Torques
  {
    // get state variables
    std::array<double, 7> coriolis_array = model.coriolis(robot_state);
    std::array<double, 42> jacobian_array =
        model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired equilibrium pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    Eigen::VectorXd tau_task(7), tau_d(7);

    // Spring damper system with damping ratio=1
    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
    tau_d << tau_task + coriolis;
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    return tau_d_array;
  };

  arm.control(impedance_control_callback);
}

const float RobotArm::determinePositionFromCommand(const float command)
{
  float pos = (command * (position_final[2] - position_start[2])) + position_start[2];
  BOOST_LOG_TRIVIAL(debug) << "New Arm Position: " << pos;

  return pos;
}

const float RobotArm::getPosition_d()
{
  return position_d[2];
}

void RobotArm::setPosition_d(const float target)
{
  BOOST_LOG_TRIVIAL(debug) << "Setting arm target to: " << target;
  position_d[2] = target;
}

void RobotArm::setTargetPosition(const float command)
{
  BOOST_LOG_TRIVIAL(debug) << "Received arm command: " << command;
  float pos = determinePositionFromCommand(command);

  // if the command position is a change of 5% or more, then move only by 5%
  float threshold_high = 1.05 * getPosition_d();
  float threshold_low = 0.95 * getPosition_d();

  // if the commanded position is larger than the high pass threshold, then over write the variable with the threshold value
  if (pos > threshold_high)
  {
    pos = threshold_high;
    BOOST_LOG_TRIVIAL(debug) << "Applying high pass filer. Commanded position is: " << pos;
  }
  // if the commanded position is smaller than the low pass threshold, then over write the variable with the threshold value
  else if (pos < threshold_low)
  {
    pos = threshold_low;
    BOOST_LOG_TRIVIAL(debug) << "Applying low pass filer. Commanded position is: " << pos;
  }

  setPosition_d(pos);
}

void RobotArm::isGraspComplete(const uint16_t &thumb, const uint16_t &mrl, const uint16_t &index)
{
  // grasp is complete if the torque sensors sense a value in the range of 900 and 800
  grasped = (index < 900) and (index > 800);
  std::cout << "Hand closed: " << grasped << "  index=" << index << std::endl;
  BOOST_LOG_TRIVIAL(debug) << "Hand closed: " << grasped;
}

void RobotArm::updateState()
{
  bool read_success;
  uint16_t thumb, mrl, index;
  readoutPosition(read_success, thumb, mrl, index);

  // if the readout was successful, update the state of `grasped` according to the new readout
  if (read_success)
  {
    isGraspComplete(thumb, mrl, index);
  }
}