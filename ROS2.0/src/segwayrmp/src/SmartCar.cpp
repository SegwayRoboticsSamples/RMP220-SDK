#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/robot.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("SmartCar");

    node->declare_parameter<std::string>("serial_full_name", "/dev/ttyUSB0");
    std::string serial_full_name;
    node->get_parameter("serial_full_name", serial_full_name);
    // init_control()parameter: Fill in the full path name of the actual serial port being used

    set_smart_car_serial((char*)serial_full_name.c_str());//If a serial port is used, set the serial port name.
    set_comu_interface(comu_serial);//Before calling init_control_ctrl, need to call this function set whether the communication port is serial or CAN.

    if (init_control_ctrl() == -1) { 
        printf("init_control failed!\n");
        exit_control_ctrl();
    } else {
        printf("init_control success!\n");
    }
    robot::Chassis sbv(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}