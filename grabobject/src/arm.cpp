#include <arm.h>

RobotArm::RobotArm(const std::string hand_serial_port,
                   const std::string arm_host_name)
    : arm(arm_host_name) {
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
}

void RobotArm::gripObject() { hand.Write("@AGSM07045++++++*\r"); }

void RobotArm::releaseObject() { hand.Write("@AGSM00045++++++*\r"); }

double RobotArm::position(double start_angle, double end_angle, double time,
                          double time_now) {
  double a_2 = 3 / (time * time) * (end_angle - start_angle);
  double a_3 = -2 / (time * time * time) * (end_angle - start_angle);

  return (a_2 * (time_now * time_now)) +
         (a_3 * (time_now * time_now * time_now));
}

void RobotArm::setDefaultBehavior() {
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

void RobotArm::moveArm(const std::array<double, 7> &movement_angles,
                       const double &movement_duration) {
  std::array<double, 7> initial_position;
  double time;

  arm.control(
      [this, &movement_duration, &movement_angles, &time,
       &initial_position](const franka::RobotState &robot_state,
                          franka::Duration period) -> franka::JointPositions {
        time += period.toSec();

        if (time == 0.0) {
          initial_position = robot_state.q_d;
        }

        double delta_shoulder = position(
            initial_position[1], movement_angles[1], movement_duration, time);
        double delta_elbow = position(initial_position[3], movement_angles[3],
                                      movement_duration, time);
        double delta_wrist = position(initial_position[5], movement_angles[5],
                                      movement_duration, time);

        franka::JointPositions output = {
            {initial_position[0], initial_position[1] + delta_shoulder,
             initial_position[2], initial_position[3] + delta_elbow,
             initial_position[4], initial_position[5] + delta_wrist,
             initial_position[6]}};

        if (time >= movement_duration) {
          std::cout << std::endl << "Finished phase " << std::endl;

          return franka::MotionFinished(output);
        }
        return output;
      });
}
