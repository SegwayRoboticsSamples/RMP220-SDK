#ifndef _ROBOT_H
#define _ROBOT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "comm_ctrl_navigation.h"
#include "segway_msgs/chassis_ctrl_src_fb.h"
#include "segway_msgs/chassis_mileage_meter_fb.h"
#include "segway_msgs/chassis_mode_fb.h"
#include "segway_msgs/error_code_fb.h"
#include "segway_msgs/motor_work_mode_fb.h"
#include "segway_msgs/speed_fb.h"
#include "segway_msgs/ticks_fb.h"
#include "segway_msgs/bms_fb.h"
#include "Ge_encoder_odometry.h"
#include "segway_msgs/ros_get_sw_version_cmd.h"
#include "segway_msgs/ros_get_vel_max_feedback_cmd.h"
#include "segway_msgs/ros_get_charge_mos_ctrl_status_cmd.h"
#include "segway_msgs/ros_set_charge_mos_ctrl_cmd.h"
#include "segway_msgs/ros_set_chassis_enable_cmd.h"
#include "segway_msgs/ros_set_chassis_poweroff_cmd.h"
#include "segway_msgs/ros_set_load_param_cmd.h"
#include "segway_msgs/ros_set_remove_push_cmd.h"
#include "segway_msgs/ros_set_vel_max_cmd.h"
#include "segway_msgs/ros_set_chassis_calib_imu_cmd.h"
#include "segway_msgs/chassis_send_event.h"
#include "segway_msgs/ros_get_load_param_cmd.h"
#include "segway_msgs/ros_get_low_power_shutdown_threshold_cmd.h"
#include "segway_msgs/ros_set_low_power_shutdown_threshold_cmd.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "segway_msgs/ros_set_iap_cmdAction.h"
#include "tf/transform_broadcaster.h"

#define IAP_STATE_COMPLETE  3
#define IAP_STATE_FAIL      4
#define IAP_STATE_ABORT     5
typedef actionlib::SimpleActionServer<segway_msgs::ros_set_iap_cmdAction> iapActionServer;
typedef actionlib::SimpleActionClient<segway_msgs::ros_set_iap_cmdAction> iapActionClient;
namespace robot
{

    class Chassis
    {
        public:
            Chassis(const ros::NodeHandle& nh);
            static void chassis_send_event_callback(int event_no);
        private:
            void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel);
            void TimeUpdate1000Hz(const ros::TimerEvent& event);
            void TimeUpdate1Hz(const ros::TimerEvent& event);
            void PubOdomToRosOdom(Odometry odom_data);
            void PubImuToRosImu(void);
        
            ros::NodeHandle nh_;

            ros::Subscriber velocity_sub_;
            
            ros::Publisher bms_fb_pub;
            ros::Publisher chassis_ctrl_src_fb_pub;
            ros::Publisher chassis_mileage_meter_fb_pub;
            ros::Publisher chassis_mode_fb_pub;
            ros::Publisher error_code_fb_pub;
            ros::Publisher motor_work_mode_fb_pub;
            ros::Publisher speed_fb_pub;
            ros::Publisher ticks_fb_pub;
            // ros::Publisher Speed_pub;
            // ros::Publisher Ticks_pub;
            // ros::Publisher Info_pub;
            // ros::Publisher Event_info_pub;
            ros::Publisher Odom_pub;
            ros::Publisher Imu_pub;
            // ros::Publisher Set_Limit_Fb_pub;
            tf::TransformBroadcaster    odom_broadcaster;

            ros::Timer update_timer_;
            ros::Timer update_timer2_;

            segway_msgs::bms_fb bms_fb;
            segway_msgs::chassis_ctrl_src_fb chassis_ctrl_src_fb;
            segway_msgs::chassis_mileage_meter_fb chassis_mileage_meter_fb;
            segway_msgs::chassis_mode_fb chassis_mode_fb;
            segway_msgs::error_code_fb error_code_fb;
            segway_msgs::motor_work_mode_fb motor_work_mode_fb;
            segway_msgs::speed_fb speed_fb;
            segway_msgs::ticks_fb ticks_fb;
            nav_msgs::Odometry ROS_odom;
            sensor_msgs::Imu ros_imu;
            geometry_msgs::TransformStamped odom_trans;


            bool ros_get_load_param_cmd_callback(segway_msgs::ros_get_load_param_cmd::Request &req, segway_msgs::ros_get_load_param_cmd::Response &res);
            bool ros_get_sw_version_cmd_callback(segway_msgs::ros_get_sw_version_cmd::Request &req, segway_msgs::ros_get_sw_version_cmd::Response &res);
            bool ros_get_vel_max_feedback_cmd_callback(segway_msgs::ros_get_vel_max_feedback_cmd::Request &req, segway_msgs::ros_get_vel_max_feedback_cmd::Response &res);
            bool ros_get_charge_mos_ctrl_status_cmd_callback(segway_msgs::ros_get_charge_mos_ctrl_status_cmd::Request &req, segway_msgs::ros_get_charge_mos_ctrl_status_cmd::Response &res);
            bool ros_set_charge_mos_ctrl_cmd_callback(segway_msgs::ros_set_charge_mos_ctrl_cmd::Request &req, segway_msgs::ros_set_charge_mos_ctrl_cmd::Response &res);
            bool ros_set_chassis_enable_cmd_callback(segway_msgs::ros_set_chassis_enable_cmd::Request &req, segway_msgs::ros_set_chassis_enable_cmd::Response &res);
            bool ros_set_chassis_poweroff_cmd_callback(segway_msgs::ros_set_chassis_poweroff_cmd::Request &req, segway_msgs::ros_set_chassis_poweroff_cmd::Response &res);
            bool ros_set_load_param_cmd_callback(segway_msgs::ros_set_load_param_cmd::Request &req, segway_msgs::ros_set_load_param_cmd::Response &res);
            bool ros_set_remove_push_cmd_callback(segway_msgs::ros_set_remove_push_cmd::Request &req, segway_msgs::ros_set_remove_push_cmd::Response &res);
            bool ros_set_vel_max_cmd_callback(segway_msgs::ros_set_vel_max_cmd::Request &req, segway_msgs::ros_set_vel_max_cmd::Response &res);
            bool ros_set_chassis_calib_imu_cmd_callback(segway_msgs::ros_set_chassis_calib_imu_cmd::Request &req, segway_msgs::ros_set_chassis_calib_imu_cmd::Response &res);
            bool ros_get_low_power_shutdown_threshold_cmd_callback(segway_msgs::ros_get_low_power_shutdown_threshold_cmd::Request &req, segway_msgs::ros_get_low_power_shutdown_threshold_cmd::Response &res);
            bool ros_set_low_power_shutdown_threshold_cmd_callback(segway_msgs::ros_set_low_power_shutdown_threshold_cmd::Request &req, segway_msgs::ros_set_low_power_shutdown_threshold_cmd::Response &res);

            ros::ServiceServer ros_get_load_param_cmd_srv_server;
            ros::ServiceServer ros_get_sw_version_cmd_srv_server;
            ros::ServiceServer ros_get_vel_max_feedback_cmd_srv_server;
            ros::ServiceServer ros_get_charge_mos_ctrl_status_cmd_srv_server;
            ros::ServiceServer ros_set_charge_mos_ctrl_cmd_srv_server;
            ros::ServiceServer ros_set_chassis_enable_cmd_srv_server;
            ros::ServiceServer ros_set_chassis_poweroff_cmd_srv_server;
            ros::ServiceServer ros_set_load_param_cmd_srv_server;
            ros::ServiceServer ros_set_remove_push_cmd_srv_server;
            ros::ServiceServer ros_set_vel_max_cmd_srv_server;
            ros::ServiceServer ros_set_chassis_calib_imu_cmd_srv_server;
            ros::ServiceServer ros_get_low_power_shutdown_threshold_cmd_srv_server;
            ros::ServiceServer ros_set_low_power_shutdown_threshold_cmd_srv_server;
            

            
            
            s_aprctrl_datastamped_t timestamp_data;
            s_aprctrl_event_t       event_data;

            //std::shared_ptr<Ge_encoder_odometry> m_ge_encoder ;
            Ge_encoder_odometry m_ge_encoder;

    };

}

#endif
