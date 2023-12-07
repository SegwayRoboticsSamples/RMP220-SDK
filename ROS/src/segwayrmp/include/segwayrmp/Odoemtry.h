#ifndef _Odoemtry_H_
#define _Odoemtry_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "segwayrmp/sensor.h"

#define pi      3.141593f

namespace odometry
{
    class Odometry
    {
        public:
            Odometry(const ros::NodeHandle& nh);

        private:
            void Odometry_Cacl(double vx,double vy,double th);
            void Sensor_callback(const segwayrmp::sensor::ConstPtr &msg);
            void TimeUpdate20Hz(const ros::TimerEvent& event);
        
            ros::NodeHandle nh_;

            ros::Subscriber Sensor_sub_;
            ros::Publisher Odom_pub_;

            ros::Timer update_timer_;
            ros::Time current_time, last_time;

            tf::TransformBroadcaster odom_broadcaster;
            nav_msgs::Odometry odom;//定义里程计对象
            geometry_msgs::Quaternion odom_quat; //四元数变量
            geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息

            int16_t L_Ticks_;
            int16_t R_Ticks_;
            int16_t Pre_L_Ticks_;
            int16_t Pre_R_Ticks_;
    };

}

#endif