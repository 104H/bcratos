#pragma once

#include <eigen3/Eigen/Core>

#include <franka/exception.h>
#include <franka/robot.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

class RobotArm
{
private:
    // mia hand port
    LibSerial::SerialPort hand;

    // franka emika arm hostname
    franka::Robot arm;

public:
    RobotArm(const std::string hand_serial_port, const std::string arm_host_name);
    void gripObject();
    void releaseObject();
    double position(double start_angle, double end_angle, double time,
                    double time_now);
}
