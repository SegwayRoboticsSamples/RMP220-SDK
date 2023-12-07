#ifndef _ROBOT_H
#define _ROBOT_H

#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include "stdint.h"
#include <time.h>
#include <sys/time.h>

#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "comm_ctrl_navigation.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "segway_msgs/msg/bms_fb.hpp"
#include "segway_msgs/msg/chassis_ctrl_src_fb.hpp"
#include "segway_msgs/msg/chassis_mileage_meter_fb.hpp"
#include "segway_msgs/msg/chassis_mode_fb.hpp"
#include "segway_msgs/msg/error_code_fb.hpp"
#include "segway_msgs/msg/motor_work_mode_fb.hpp"
#include "segway_msgs/msg/speed_fb.hpp"
#include "segway_msgs/msg/ticks_fb.hpp"
#include "segway_msgs/srv/chassis_send_event.hpp"
#include "segway_msgs/srv/ros_get_charge_mos_ctrl_status_cmd.hpp"
#include "segway_msgs/srv/ros_get_load_param_cmd.hpp"
#include "segway_msgs/srv/ros_get_sw_version_cmd.hpp"
#include "segway_msgs/srv/ros_get_vel_max_feedback_cmd.hpp"
#include "segway_msgs/srv/ros_set_charge_mos_ctrl_cmd.hpp"
#include "segway_msgs/srv/ros_set_chassis_enable_cmd.hpp"
#include "segway_msgs/srv/ros_set_chassis_calib_imu_cmd.hpp"
#include "segway_msgs/srv/ros_set_chassis_poweroff_cmd.hpp"
#include "segway_msgs/srv/ros_set_load_param_cmd.hpp"
#include "segway_msgs/srv/ros_set_remove_push_cmd.hpp"
#include "segway_msgs/srv/ros_set_vel_max_cmd.hpp"
#include "segway_msgs/srv/ros_get_low_power_shutdown_threshold_cmd.hpp"
#include "segway_msgs/srv/ros_set_low_power_shutdown_threshold_cmd.hpp"
#include "segway_msgs/action/ros_set_iap_cmd.hpp"

#define pi   3.141592654
#define pi_2 6.283185308

namespace robot
{
    class Chassis
    {
        public:
            using iapCmd = segway_msgs::action::RosSetIapCmd;
            using goalHandaleIapCmd = rclcpp_action::ServerGoalHandle<iapCmd>;
            Chassis(rclcpp::Node::SharedPtr nh);
            static void pub_event_callback(int event_no);
        private:
            std::shared_ptr<rclcpp::Node> node;

            std::string bins_directory;
            std::string central_version;
            std::string motor_version;

            tf2::Quaternion odom_quat;
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> odom_broadcaster;

            segway_msgs::msg::BmsFb bms_fb;
            segway_msgs::msg::ChassisCtrlSrcFb chassis_ctrl_src_fb;
            segway_msgs::msg::ChassisMileageMeterFb chassis_mileage_meter_fb;
            segway_msgs::msg::ChassisModeFb chassis_mode_fb;
            segway_msgs::msg::ErrorCodeFb error_code_fb;
            segway_msgs::msg::MotorWorkModeFb motor_work_mode_fb;
            segway_msgs::msg::SpeedFb speed_fb;
            segway_msgs::msg::TicksFb ticks_fb;
            geometry_msgs::msg::TransformStamped odom_trans;
            nav_msgs::msg::Odometry odom_fb;
            sensor_msgs::msg::Imu imu_fb;     

            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub;

            rclcpp::Publisher<segway_msgs::msg::BmsFb>::SharedPtr bms_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::ChassisCtrlSrcFb>::SharedPtr chassis_ctrl_src_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::ChassisMileageMeterFb>::SharedPtr chassis_mileage_meter_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::ChassisModeFb>::SharedPtr chassis_mode_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::ErrorCodeFb>::SharedPtr error_code_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::MotorWorkModeFb>::SharedPtr motor_work_mode_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::SpeedFb>::SharedPtr speed_fb_pub;
            rclcpp::Publisher<segway_msgs::msg::TicksFb>::SharedPtr ticks_fb_pub;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;


            rclcpp::Service<segway_msgs::srv::RosGetChargeMosCtrlStatusCmd>::SharedPtr ros_get_charge_mos_ctrl_status_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosGetLoadParamCmd>::SharedPtr ros_get_load_param_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosGetSwVersionCmd>::SharedPtr ros_get_sw_version_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosGetVelMaxFeedbackCmd>::SharedPtr ros_get_vel_max_feedback_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetChargeMosCtrlCmd>::SharedPtr ros_set_charge_mos_ctrl_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetChassisEnableCmd>::SharedPtr ros_set_chassis_enable_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetChassisCalibImuCmd>::SharedPtr ros_set_chassis_calib_imu_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetChassisPoweroffCmd>::SharedPtr ros_set_chassis_poweroff_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetLoadParamCmd>::SharedPtr ros_set_load_param_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetRemovePushCmd>::SharedPtr ros_set_remove_push_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetVelMaxCmd>::SharedPtr ros_set_vel_max_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosGetLowPowerShutdownThresholdCmd>::SharedPtr ros_get_low_power_shutdown_threshold_cmd_server;
            rclcpp::Service<segway_msgs::srv::RosSetLowPowerShutdownThresholdCmd>::SharedPtr ros_set_low_power_shutdown_threshold_cmd_server;

            void ros_get_charge_mos_ctrl_status_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetChargeMosCtrlStatusCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosGetChargeMosCtrlStatusCmd::Response> response);
            void ros_get_load_param_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetLoadParamCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosGetLoadParamCmd::Response> response);
            void ros_get_sw_version_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetSwVersionCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosGetSwVersionCmd::Response> response);
            void ros_get_vel_max_feedback_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetVelMaxFeedbackCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosGetVelMaxFeedbackCmd::Response> response);
            void ros_set_charge_mos_ctrl_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChargeMosCtrlCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetChargeMosCtrlCmd::Response> response);
            void ros_set_chassis_enable_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChassisEnableCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetChassisEnableCmd::Response> response);
            void ros_set_chassis_calib_imu_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChassisCalibImuCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetChassisCalibImuCmd::Response> response);
            void ros_set_chassis_poweroff_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChassisPoweroffCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetChassisPoweroffCmd::Response> response);
            void ros_set_load_param_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetLoadParamCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetLoadParamCmd::Response> response);
            void ros_set_remove_push_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetRemovePushCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetRemovePushCmd::Response> response);
            void ros_set_vel_max_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetVelMaxCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetVelMaxCmd::Response> response);
            void ros_get_low_power_shutdown_threshold_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetLowPowerShutdownThresholdCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosGetLowPowerShutdownThresholdCmd::Response> response);
            void ros_set_low_power_shutdown_threshold_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetLowPowerShutdownThresholdCmd::Request> request,
                std::shared_ptr<segway_msgs::srv::RosSetLowPowerShutdownThresholdCmd::Response> response);

            rclcpp_action::Server<iapCmd>::SharedPtr iap_action_server;

            rclcpp::TimerBase::SharedPtr timer_1hz;
            rclcpp::TimerBase::SharedPtr timer_1000hz;

            s_aprctrl_datastamped_t timestamp_data;
            s_aprctrl_event_t       event_data;

            void timer_1000hz_callback(void);
            void timer_1hz_callback(void);
            void speed_pub_callback(void);
            void ticks_pub_callback(void);
            void imu_pub_callback(void);
            void pub_odom_callback(void);
            void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;

            void iapCmdExecute(const std::shared_ptr<goalHandaleIapCmd> goal_handle);
            rclcpp_action::GoalResponse handle_iapCmdGoal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const iapCmd::Goal> goal)
            {
                RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "Receive goal request with iap_board %d", goal->iap_board);
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse handle_iapCmdCancel(
                const std::shared_ptr<goalHandaleIapCmd> goal_handle)
            {
                RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void handle_iapCmdAccepted(const std::shared_ptr<goalHandaleIapCmd> goal_handle)
            {
                using namespace std::placeholders;
                (void)goal_handle;
                std::thread{std::bind(&Chassis::iapCmdExecute, this, _1), goal_handle}.detach();
            }
    };

}

#endif
