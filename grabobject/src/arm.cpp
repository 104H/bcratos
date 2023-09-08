#include <arm.h>

RobotArm::RobotArm(const std::string hand_serial_port, const std::string arm_host_name)
{
    arm = robot(arm_host_name);

    hand.Open(hand_serial_port);
    // Set the baud rate.
    hand.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void RobotArm::gripObject()
{
    hand.Write("@AGSM07045++++++*\r");
}

void RobotArm::releaseObject()
{
    hand.Write("@AGSM00045++++++*\r");
}

double RobotArm::position(double start_angle, double end_angle, double time,
                          double time_now)
{
    // double a_0 = start_angle;
    double a_2 = 3 / (time * time) * (end_angle - start_angle);
    double a_3 = -2 / (time * time * time) * (end_angle - start_angle);

    return (a_2 * (time_now * time_now)) +
           (a_3 * (time_now * time_now * time_now));
}