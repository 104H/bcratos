#include <arm.h>

RobotArm::RobotArm(const std::string hand_serial_port,
                   const std::string arm_host_name)
    : arm(arm_host_name)
{
  // franka::Robot arm(arm_host_name);

  hand.Open(hand_serial_port);
  // Set the baud rate.
  hand.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

  // define the target position
  franka::RobotState initial_state = arm.readOnce();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  Eigen::Vector3d position_d(initial_transform.translation());
  Eigen::Quaterniond orientation_d(initial_transform.linear());

  arm.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                           {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                           {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                           {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                           {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                           {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                           {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                           {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  arm.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  arm.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  moveToStart();
}

void RobotArm::gripObject() { hand.Write("@AGSM07045++++++*\r"); }

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
  arm.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                           {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                           {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                           {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                           {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                           {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                           {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                           {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  arm.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  arm.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
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
      std::cout << std::endl << "Finished phase " << std::endl;

      return franka::MotionFinished(output);
    }
    return output; });
}

const std::array<double, 7> RobotArm::scaleAngles(std::array<double, 7> start_angles, std::array<double, 7> end_angles, const float extent)
{
  std::array<double, 7> scaled_angles;
  scaled_angles[0] = end_angles[0];
  scaled_angles[2] = end_angles[2];
  scaled_angles[4] = end_angles[4];
  scaled_angles[6] = end_angles[6];

  scaled_angles[1] = (extent * (end_angles[1] - start_angles[1])) + start_angles[1];
  scaled_angles[5] = (extent * (end_angles[5] - start_angles[5])) + start_angles[5];

  return scaled_angles;
}

void RobotArm::reachAndGrab(float const extent)
{
  std::array<double, 7> initial_position;
  double time;

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

  for (int i = 0; i < 2; i++)
  {
    double time = 0.0, duration;

    // load robot model
    franka::Model model = arm.loadModel();

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

    // if the command was not to reach the final position
    if (extent < 1.0)
    {
      return;
    }

    if (i == 0)
    {
      // arm has reached the object, grab object
      std::cout << std::endl
                << "Grabbing Object" << std::endl;
      gripObject();
    }
    else if (i == 1)
    {
      // arm has lifted the object, release object
      std::cout << std::endl
                << "Releasing Object" << std::endl;
      releaseObject();
    }
  }
}

const double RobotArm::computeExtent(const std::array<double, 7> angles)
{
  return (angles[1] - movement_angles[1][1]) / (movement_angles[0][1] - movement_angles[1][1]);
}