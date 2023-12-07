//
// Created by efan on 3/27/20.
//

#ifndef GE_ENCODER_ANSYS_GE_ENCODER_SENSORDATA_H
#define GE_ENCODER_ANSYS_GE_ENCODER_SENSORDATA_H

#include <iostream>
struct SensorData{
    SensorData(){};

    struct BaseImu{
        int64_t  TimeStamp = -1;
        double baseYaw = 0;
        BaseImu(){};
        BaseImu(int64_t timeStamp_, double baseyaw_) : TimeStamp(timeStamp_), baseYaw(baseyaw_){}
        BaseImu& operator=(BaseImu& other)
        {
            if(this != &other) {
                this->baseYaw = other.baseYaw;
                this->TimeStamp = other.TimeStamp;
            }
            return *this;
        }

        BaseImu Interpolation(int64_t time_inter, BaseImu imu_before, BaseImu imu_after){
//            static int num = 0;

            if(time_inter <= imu_before.TimeStamp)return imu_before;
            if(time_inter >= imu_after.TimeStamp)return imu_after;
            if(imu_before.TimeStamp == imu_after.TimeStamp)return imu_before;
            BaseImu res;
//            std::cout << "interpolation times: " << num ++ << std::endl;
            res.TimeStamp = time_inter;
            double scale = double(time_inter - imu_before.TimeStamp)/double(imu_after.TimeStamp - imu_before.TimeStamp);
            res.baseYaw = imu_before.baseYaw +  (imu_after.baseYaw - imu_before.baseYaw)*scale;
            // std::cout << imu_before.baseYaw << "," << res.baseYaw << "," << imu_after.baseYaw << std::endl;
            return res;
        }
    } baseImu;

    struct Ticks{
        int64_t  TimeStamp = -1;
        int leftTicks = 0;
        int rightTicks = 0;
        Ticks(){}
        Ticks(int64_t timeStamp_, int leftticks_, int rightticks_): TimeStamp(timeStamp_), leftTicks(leftticks_), rightTicks(rightticks_){}
    } ticks;
};
struct Odometry{
    int64_t  TimeStamp = -1;
    /**
     * Autor: Efan
     * translation:
     *      nav_msg::Odometry o->time <- TimeStamp
     *      nav_msg::Odometry o->pose->pose->position(pose_.x,pose_.y,0)
     *      nav_msg::Odometry o->pose->pose->orientation(0,0,sin(pose_.orientation/2),cos(pose_.orientation/2))
     *      nav_msg::Odometry o->twist->twist->linear(v_x, v_y, 0)
     *      nav_msg::Odometry o->twist->twist->angular(0, 0, w_z)
    */
    struct Pose{
//        int64_t TimeStamp = -1;
        double x = 0;
        double y = 0;
        double orientation = 0;
        Pose(){}
        Pose(double x_, double y_, double orientation_):x(x_), y(y_), orientation(orientation_){}
    } pose_;
    struct Twist{
//        int64_t TimeStamp = -1;
        double v_x = 0;
        double v_y = 0;
        double w_z = 0;
        Twist(){}
        Twist(double v_x_, double v_y_, double w_z_): v_x(v_x_), v_y(v_y_), w_z(w_z_){}
    } twist_;
};


#endif //GE_ENCODER_ANSYS_GE_ENCODER_SENSORDATA_H
