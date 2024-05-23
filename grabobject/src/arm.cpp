#include <arm.h>

RobotArm::RobotArm(const std::string hand_serial_port,
                   const std::string arm_host_name)
    : arm(arm_host_name)
{
  // franka::Robot arm(arm_host_name);

  hand.Open(hand_serial_port);
  // Set the baud rate.
  hand.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

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

  for (int i = 0; i < 2; i++)
  {
    double time = 0.0, duration;
    std::array<double, 7> angles;

    if (i == 0)
    {
      // determine robot pose
      std::array<double, 7> robot_state = arm.readOnce().q;

      // determine current extent based on current angles
      double current_extent = computeExtent(robot_state);

      // update extent according to last extent
      angles = scaleAngles(movement_angles[1], movement_angles[i], extent);
      duration = std::abs(current_extent - extent) * movement_duration[i];
    }
    else
    {
      angles = movement_angles[i];
      duration = movement_duration[i];
    }

    arm.control([this, &time, &initial_position, &angles,
                 &duration](const franka::RobotState &robot_state,
                            franka::Duration period) -> franka::JointPositions
                {
      time += period.toSec();

      if (time == 0.0) {
        initial_position = robot_state.q_d;
        return franka::JointPositions(initial_position);
      }

      double delta_shoulder =
          position(initial_position[1], angles[1], duration, time);
      double delta_elbow =
          position(initial_position[3], angles[3], duration, time);
      double delta_wrist =
          position(initial_position[5], angles[5], duration, time);

      franka::JointPositions output = {
          {initial_position[0], initial_position[1] + delta_shoulder,
           initial_position[2], initial_position[3] + delta_elbow,
           initial_position[4], initial_position[5] + delta_wrist,
           initial_position[6]}};

      if (time >= duration) {
        std::cout << std::endl << "Finished phase " << std::endl;

        return franka::MotionFinished(output);
      }
      return output; });

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