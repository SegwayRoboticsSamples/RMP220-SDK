#include "ros/ros.h"
#include "segwayrmp/robot.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "SmartCar");
    ros::NodeHandle n;

    if (n.hasParam("segwaySmartCarSerial")){
        std::string nn;
        n.getParam("segwaySmartCarSerial", nn);
        printf("segwaySmartCarSerial: %s\n",nn.c_str());
        set_smart_car_serial(nn.c_str());
    }
    
    //Before calling init_control_ctrl, need to call this function to set whether the communication port is a serial port or a CAN port, "comu)serial":serial; "comu_can":CAN.;Others: Illegal
     set_comu_interface(comu_serial); 
    if(init_control_ctrl() == -1){
        printf("init_control faild\n");
    }else{
        printf("init success!\n");
    }  

    robot::Chassis sbv(n);

    ros::spin();
    return 0;
}
