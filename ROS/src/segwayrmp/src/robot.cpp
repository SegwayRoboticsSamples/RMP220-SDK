#include "segwayrmp/robot.h"
#include <sensor_msgs/Imu.h>
#include "Ge_encoder_odometry.h"
#if 1
#define ODOM_BY_CHASSIS

#define IMU_ANGULAR_VEL_CONVERT_UINIT 0.0009288 //(2000 /32767 / 0.0163835 / 70 / 57.3)//FS:2000dps; 0.0163835/70：coefficient；57.3：rad->degree
#define IMU_LINEAR_VEL_CONVERT_UINIT 0.0023943  //(9.80665 * 8 /32767 )//FS:8G;
#define RAD_DEGREE_CONVER            57.2958
chassis_speed_data_t SpeedData;
uint64_t Speed_TimeStamp;
uint8_t Speed_update;

motor_ticks_t TicksData;
uint64_t Ticks_TimeStamp;
uint8_t Ticks_update;

imu_gyr_original_data_t ImuGyrData;
uint64_t ImuGyr_TimeStamp;
uint8_t ImuGyr_update;

imu_acc_original_data_t ImuAccData;
uint64_t ImuAcc_TimeStamp;
uint8_t ImuAcc_update;

odom_pos_xy_t OdomPoseXy;
odom_euler_xy_t OdomEulerXy;
odom_euler_z_t OdomEulerZ;
odom_vel_line_xy_t OdomVelLineXy;
uint64_t Odom_TimeStamp;
uint8_t Odom_update;

int chassis_event_id = 0;
ros::ServiceClient chassis_send_event_srv_client;

static void PubData(StampedBasicFrame *frame)
{
    if (frame->type_id == Chassis_Data_Speed)
    {
        memcpy(&SpeedData, frame->data, sizeof(SpeedData)); //Speed data from chassis
        Speed_TimeStamp = frame->timestamp;
        Speed_update = 1;
    }
    else if (frame->type_id == Chassis_Data_Ticks)
    {
        memcpy(&TicksData, frame->data, sizeof(TicksData)); //Ticks data from chassis
        Ticks_TimeStamp = frame->timestamp;
        Ticks_update = 1;
        // ROS_INFO("LTICKS:%d, rticks:%d", TicksData.l_ticks, TicksData.r_ticks);
    }
    else if (frame->type_id == Chassis_Data_Imu_Gyr)
    {
        memcpy(&ImuGyrData, frame->data, sizeof(ImuGyrData)); //Ticks data from chassis
        ImuGyr_TimeStamp = frame->timestamp;
        ImuGyr_update = 1;
        // ROS_INFO("GYR0:%d, gyr1:%d, gyr2:%d", ImuGyrData.gyr[0], ImuGyrData.gyr[1], ImuGyrData.gyr[2]);
    }
    else if (frame->type_id == Chassis_Data_Imu_Acc)
    {
        memcpy(&ImuAccData, frame->data, sizeof(ImuAccData)); //Ticks data from chassis
        ImuAcc_TimeStamp = frame->timestamp;
        ImuAcc_update = 1;
        // ROS_INFO("ACC0:%d, acc1:%d, acc2:%d", ImuAccData.acc[0], ImuAccData.acc[1], ImuAccData.acc[2]);
    }
    else if (frame->type_id == Chassis_Data_Odom_Pose_xy)
    {
        memcpy(&OdomPoseXy, frame->data, sizeof(OdomPoseXy)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 1;
        // ROS_INFO("odomPosX:%f, odomPosY:%f", OdomPoseXy.pos_x, OdomPoseXy.pos_y);
    }
    else if (frame->type_id == Chassis_Data_Odom_Euler_xy)
    {
        memcpy(&OdomEulerXy, frame->data, sizeof(OdomEulerXy)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 2;
        // ROS_INFO("OdomEulerX:%f, OdomEulerY:%f", OdomEulerXy.euler_x, OdomEulerXy.euler_y);
    }
    else if (frame->type_id == Chassis_Data_Odom_Euler_z)
    {
        memcpy(&OdomEulerZ, frame->data, sizeof(OdomEulerZ)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 4;
        // ROS_INFO("OdomEulerZ:%f", OdomEulerZ.euler_z);
    }
    else if (frame->type_id == Chassis_Data_Odom_Linevel_xy)
    {
        memcpy(&OdomVelLineXy, frame->data, sizeof(OdomVelLineXy)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 8;
        // ROS_INFO("OdomVelLineX:%f, OdomVelLineY:%f", OdomVelLineXy.vel_line_x, OdomVelLineXy.vel_line_y);
    }
}

static void EvnetPubData(int event_no)
{
    chassis_event_id = event_no;
    robot::Chassis::chassis_send_event_callback(event_no);
}

//The timestamp from the upper computer is the count of microseconds，
ros::Time timestamp2rostime(int64_t timestamp)
{
    //    std::string suanz = std::to_string(timestamp);
    //    std::string sec_string = suanz.substr(0,10);
    //    std::string nsec_string = suanz.substr(10,9);
    //    while(nsec_string.length() < 9){
    //        nsec_string += "0";
    //    }
    //    return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
    uint32_t sec_ = timestamp / 1000000;
    uint32_t nsec_ = (timestamp % 1000000) * 1000;
    return ros::Time(sec_, nsec_);
}

double getOrientationX()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw);
}

double getOrientationY()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw);
}

double getOrientationZ()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw);
}

double getOrientationW()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw);
}

void ros_set_iap_cmd_callback(const segway_msgs::ros_set_iap_cmdGoalConstPtr &ros_set_iap_cmd_goal, iapActionServer *as)
{
    if (ros_set_iap_cmd_goal->central_board_iap_enable == false)
    {
        ROS_INFO("ros_set_iap_cmd_goal->central_board_iap_enable false:%d ", ros_set_iap_cmd_goal->central_board_iap_enable);
        return;
    }

    ROS_INFO("ros_set_iap_cmd_goal->central_board_iap_enable, true:%d ", ros_set_iap_cmd_goal->central_board_iap_enable);
    ros::Rate loop_time(1);
    segway_msgs::ros_set_iap_cmdFeedback iap_fb;
    segway_msgs::ros_set_iap_cmdResult   iap_ret;

    iapCentralBoard();

    while (!isHostIapOver())
    {
        if (as->isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("ros_set_iap_cmd_action: Preempted");
            as->setPreempted();
            setHostIapCanceled();
            break;
        }
        else
        {
            iap_fb.iap_percent = getIapTotalProgress();
            as->publishFeedback(iap_fb);
        }      
        loop_time.sleep();
    }

    iap_ret.iap_result = getHostIapResult();
    iap_ret.error_code = getHostIapErrorCode();
    if (iap_ret.iap_result == IAP_STATE_COMPLETE)
    {
        as->setSucceeded(iap_ret);
    }
    else
    {
        as->setAborted(iap_ret);
    }
}
namespace robot
{

Chassis::Chassis(const ros::NodeHandle &nh) : nh_(nh)
{
    timestamp_data.on_new_data = PubData;
    aprctrl_datastamped_jni_register(&timestamp_data);

    event_data.event_callback = EvnetPubData;
    aprctrl_eventcallback_jni_register(&event_data);

    velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Chassis::cmd_vel_callback, this);

    bms_fb_pub = nh_.advertise<segway_msgs::bms_fb>("bms_fb", 10);
    chassis_ctrl_src_fb_pub = nh_.advertise<segway_msgs::chassis_ctrl_src_fb>("chassis_ctrl_src_fb", 10);
    chassis_mileage_meter_fb_pub = nh_.advertise<segway_msgs::chassis_mileage_meter_fb>("chassis_mileage_meter_fb", 10);
    chassis_mode_fb_pub = nh_.advertise<segway_msgs::chassis_mode_fb>("chassis_mode_fb", 10);
    error_code_fb_pub = nh_.advertise<segway_msgs::error_code_fb>("error_code_fb", 10);
    motor_work_mode_fb_pub = nh_.advertise<segway_msgs::motor_work_mode_fb>("motor_work_mode_fb", 10);
    speed_fb_pub = nh_.advertise<segway_msgs::speed_fb>("speed_fb", 10);
    ticks_fb_pub = nh_.advertise<segway_msgs::ticks_fb>("ticks_fb", 10);
    Odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    Imu_pub = nh_.advertise<sensor_msgs::Imu>("imu", 1);

    chassis_send_event_srv_client = nh_.serviceClient<segway_msgs::chassis_send_event>("chassis_send_event_srv");

    ros_get_load_param_cmd_srv_server = nh_.advertiseService("ros_get_load_param_cmd_srv", &Chassis::ros_get_load_param_cmd_callback, this);
    ros_get_sw_version_cmd_srv_server = nh_.advertiseService("ros_get_sw_version_cmd_srv", &Chassis::ros_get_sw_version_cmd_callback, this);
    ros_get_vel_max_feedback_cmd_srv_server = nh_.advertiseService("ros_get_vel_max_feedback_cmd_srv", &Chassis::ros_get_vel_max_feedback_cmd_callback, this);
    ros_get_charge_mos_ctrl_status_cmd_srv_server = nh_.advertiseService("ros_get_charge_mos_ctrl_status_cmd_srv", &Chassis::ros_get_charge_mos_ctrl_status_cmd_callback, this);
    ros_set_charge_mos_ctrl_cmd_srv_server = nh_.advertiseService("ros_set_charge_mos_ctrl_cmd_srv", &Chassis::ros_set_charge_mos_ctrl_cmd_callback, this);
    ros_set_chassis_enable_cmd_srv_server = nh_.advertiseService("ros_set_chassis_enable_cmd_srv", &Chassis::ros_set_chassis_enable_cmd_callback, this);
    ros_set_chassis_poweroff_cmd_srv_server = nh_.advertiseService("ros_set_chassis_poweroff_cmd_srv", &Chassis::ros_set_chassis_poweroff_cmd_callback, this);
    ros_set_load_param_cmd_srv_server = nh_.advertiseService("ros_set_load_param_cmd_srv", &Chassis::ros_set_load_param_cmd_callback, this);
    ros_set_remove_push_cmd_srv_server = nh_.advertiseService("ros_set_remove_push_cmd_srv", &Chassis::ros_set_remove_push_cmd_callback, this);
    ros_set_vel_max_cmd_srv_server = nh_.advertiseService("ros_set_vel_max_cmd_srv", &Chassis::ros_set_vel_max_cmd_callback, this);
    ros_set_chassis_calib_imu_cmd_srv_server = nh_.advertiseService("ros_set_chassis_calib_imu_cmd_srv", &Chassis::ros_set_chassis_calib_imu_cmd_callback, this);
    ros_get_low_power_shutdown_threshold_cmd_srv_server = nh_.advertiseService("ros_get_low_power_shutdown_threshold_cmd_srv", &Chassis::ros_get_low_power_shutdown_threshold_cmd_callback, this);
    ros_set_low_power_shutdown_threshold_cmd_srv_server = nh_.advertiseService("ros_set_low_power_shutdown_threshold_cmd_srv", &Chassis::ros_set_low_power_shutdown_threshold_cmd_callback, this);

    iapActionServer ros_set_iap_cmd_server(nh_, "ros_set_iap_cmd_action", boost::bind(&ros_set_iap_cmd_callback, _1, &ros_set_iap_cmd_server), false);
    ros_set_iap_cmd_server.start();
    ROS_INFO("ros_set_iap_cmd_action started");
    update_timer_ = nh_.createTimer(ros::Duration(0.001), &Chassis::TimeUpdate1000Hz, this);
    update_timer2_ = nh_.createTimer(ros::Duration(1), &Chassis::TimeUpdate1Hz, this);
    ros::spin();
}

/* code */
void Chassis::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_input)
{
    double angular_vel_ = cmd_input->angular.z; //get angular velocity of /cmd_vel,rad/s
    double linear_vel_ = cmd_input->linear.x;   //get linear velocity of /cmd_vel.m/s

    set_cmd_vel(linear_vel_, angular_vel_); //Configure coefficients according to chassis parameters

    //ROS_INFO("angular_temp:%f  rad/s   linear_temp:%f  m/s", angular_vel_,linear_vel_);
}

bool Chassis::ros_get_load_param_cmd_callback(segway_msgs::ros_get_load_param_cmd::Request &req, segway_msgs::ros_get_load_param_cmd::Response &res)
{
    if (req.ros_get_load_param == false)
    {
        return false;
    }
    res.get_load_param = get_chassis_load_state();
    ROS_INFO("get_load_param:%d ", res.get_load_param);
    return true;
}
bool Chassis::ros_get_sw_version_cmd_callback(segway_msgs::ros_get_sw_version_cmd::Request &req, segway_msgs::ros_get_sw_version_cmd::Response &res)
{
    if (req.ros_get_sw_version_cmd == false)
    {
        return false;
    }
    res.host_version = get_host_version();
    res.central_version = get_chassis_central_version();
    res.motor_version = get_chassis_motor_version();
    ROS_INFO("req.ros_get_sw_version_cmd:%d", req.ros_get_sw_version_cmd);
    ROS_INFO("res.host_version: %d, res.central_version: %d, res.motor_version: %d ", res.host_version, res.central_version, res.motor_version);
    return true;
}
bool Chassis::ros_get_vel_max_feedback_cmd_callback(segway_msgs::ros_get_vel_max_feedback_cmd::Request &req, segway_msgs::ros_get_vel_max_feedback_cmd::Response &res)
{
    if (req.ros_get_vel_max_fb_cmd == false)
    {
        return false;
    }
    res.forward_max_vel_fb = get_line_forward_max_vel_fb();
    res.backward_max_vel_fb = get_line_backward_max_vel_fb();
    res.angular_max_vel_fb = get_angular_max_vel_fb();
    ROS_INFO("req.ros_get_vel_max_fb_cmd:%d ", req.ros_get_vel_max_fb_cmd);
    ROS_INFO("res.forward_max_vel_fb： %d, res.backward_max_vel_fb： %d, res.angular_max_vel_fb： %d ", res.forward_max_vel_fb, res.backward_max_vel_fb, res.angular_max_vel_fb);
    return true;
}
bool Chassis::ros_get_charge_mos_ctrl_status_cmd_callback(segway_msgs::ros_get_charge_mos_ctrl_status_cmd::Request &req, segway_msgs::ros_get_charge_mos_ctrl_status_cmd::Response &res)
{
    int16_t ret = 0;
    if (req.ros_get_chassis_charge_ctrl_status == true)
    {
        ret = get_charge_mos_ctrl_status();
        res.chassis_charge_ctrl_status = ret;
        ROS_INFO("req.ros_get_chassis_charge_ctrl_status:%d, get_charge_mos_ctrl_status():%d ", req.ros_get_chassis_charge_ctrl_status, ret);
        return true;
    }
    else
    {
        ROS_INFO("req.ros_get_chassis_charge_ctrl_status:%d, get_charge_mos_ctrl_status():%d ", req.ros_get_chassis_charge_ctrl_status, ret);
        return false;
    }   
}
bool Chassis::ros_set_charge_mos_ctrl_cmd_callback(segway_msgs::ros_set_charge_mos_ctrl_cmd::Request &req, segway_msgs::ros_set_charge_mos_ctrl_cmd::Response &res)
{
    uint8_t ret;
    if (req.ros_set_chassis_charge_ctrl == false)
    {
        ret = set_charge_mos_ctrl(false);
    }
    else 
    {
        ret = set_charge_mos_ctrl(true);
    }
    ROS_INFO("req.ros_set_chassis_charge_ctrl:%d, set_charge_mos_ctrl():%d ", req.ros_set_chassis_charge_ctrl, ret);

    res.chassis_set_charge_ctrl_result = ret;
    return true;
}
bool Chassis::ros_set_chassis_enable_cmd_callback(segway_msgs::ros_set_chassis_enable_cmd::Request &req, segway_msgs::ros_set_chassis_enable_cmd::Response &res)
{
    uint8_t ret;
    if (req.ros_set_chassis_enable_cmd == false)
    {
        ret = set_enable_ctrl(0);
    }
    else 
    {
        ret = set_enable_ctrl(1);
    }
    ROS_INFO("req.ros_set_chassis_enable_cmd:%d, set_enable_ctrl():%d ", req.ros_set_chassis_enable_cmd, ret);

    res.chassis_set_chassis_enable_result = ret;
    return true;
}
bool Chassis::ros_set_chassis_poweroff_cmd_callback(segway_msgs::ros_set_chassis_poweroff_cmd::Request &req, segway_msgs::ros_set_chassis_poweroff_cmd::Response &res)
{
    if (req.ros_set_chassis_poweroff_cmd == false)
    {
        return false;
    }
    uint8_t ret = set_chassis_poweroff();
    ROS_INFO("req.ros_set_chassis_poweroff_cmd:%d, set_chassis_poweroff():%d ", req.ros_set_chassis_poweroff_cmd, ret);

    res.chassis_set_poweroff_result = ret;
    return true;
}
bool Chassis::ros_set_load_param_cmd_callback(segway_msgs::ros_set_load_param_cmd::Request &req, segway_msgs::ros_set_load_param_cmd::Response &res)
{
    int16_t value = req.ros_set_load_param;
    if (value != 0 && value != 1)
    {
        return false;
    }
    uint8_t ret = set_chassis_load_state(value);
    ROS_INFO("req.ros_set_load_param:%d, set_chassis_load_state():%d ", req.ros_set_load_param, ret);

    res.chassis_set_load_param_result = ret;
    return true;
}
bool Chassis::ros_set_remove_push_cmd_callback(segway_msgs::ros_set_remove_push_cmd::Request &req, segway_msgs::ros_set_remove_push_cmd::Response &res)
{
    if (req.ros_set_remove_push_cmd == false)
    {
        return false;
    }
    uint8_t ret = set_remove_push_cmd();
    ROS_INFO("req.ros_set_remove_push_cmd:%d, set_remove_push_cmd():%d ", req.ros_set_remove_push_cmd, ret);

    res.chassis_set_revove_push_result = ret;
    return true;
}
bool Chassis::ros_set_vel_max_cmd_callback(segway_msgs::ros_set_vel_max_cmd::Request &req, segway_msgs::ros_set_vel_max_cmd::Response &res)
{
    double forward = req.ros_set_forward_max_vel;
    double backward = req.ros_set_backward_max_vel;
    double angular = req.ros_set_angular_max_vel;
    uint8_t ret_forw = set_line_forward_max_vel(forward);
    uint8_t ret_back = set_line_backward_max_vel(backward);
    uint8_t ret_angl = set_angular_max_vel(angular);
    ROS_INFO("req.ros_set_forward_max_vel:%f, req.ros_set_backward_max_vel:%f, req.ros_set_angular_max_vel:%f ", forward, backward, angular);
    ROS_INFO("set_line_forward_max_vel():%d, set_line_backward_max_vel():%d, set_angular_max_vel():%d ", ret_forw, ret_back, ret_angl);

    res.chassis_set_max_vel_result = ret_forw | ret_back | ret_angl;
    return true;
}
bool Chassis::ros_set_chassis_calib_imu_cmd_callback(segway_msgs::ros_set_chassis_calib_imu_cmd::Request &req, segway_msgs::ros_set_chassis_calib_imu_cmd::Response &res)
{
    int8_t calib_ret = set_calib_gyro();
    res.chassis_calib_imu_result = calib_ret;
    ROS_INFO("res.chassis_calib_imu_result:%d[0:success; -1:fail to send cmd; -2:fail to calib; -3: overtime] ", calib_ret);
    return true;
}
bool Chassis::ros_get_low_power_shutdown_threshold_cmd_callback(segway_msgs::ros_get_low_power_shutdown_threshold_cmd::Request &req, segway_msgs::ros_get_low_power_shutdown_threshold_cmd::Response &res)
{
    uint16_t threshold_soc = 0;
    threshold_soc = get_low_power_shutdown_threshold();
    res.chassis_get_soc_threshold_result = threshold_soc;
    ROS_INFO("res.chassis_get_soc_threshold_result:[%d], get_low_power_shutdown_threshold():[%d] ", res.chassis_get_soc_threshold_result,threshold_soc);
    return true;
}
bool Chassis::ros_set_low_power_shutdown_threshold_cmd_callback(segway_msgs::ros_set_low_power_shutdown_threshold_cmd::Request &req, segway_msgs::ros_set_low_power_shutdown_threshold_cmd::Response &res)
{
    uint8_t threshold_ret = set_low_power_shutdown_threshold(req.ros_set_low_power_shutdown_threshold);
    res.chassis_set_soc_threshold_result = threshold_ret;
    ROS_INFO("req.ros_set_low_power_shutdown_threshold: [%d]", req.ros_set_low_power_shutdown_threshold);
    ROS_INFO("res.chassis_set_max_vel_result:%d[0:success; -1:fail to send cmd; -2:fail to calib; -3: overtime] ", threshold_ret);
    return true;
}

void Chassis::chassis_send_event_callback(int event_no)
{
    segway_msgs::chassis_send_event chassis_send_event_srv;

    chassis_send_event_srv.request.chassis_send_event_id = (int16_t)event_no;
    if (chassis_send_event_srv_client.call(chassis_send_event_srv))
    {
        ROS_INFO("send chassis_send_event_srv, chassis_send_event_srv.request.chassis_send_event_id: %d", chassis_send_event_srv.request.chassis_send_event_id);
        ROS_INFO("receive chassis_send_event_srv, chassis_send_event_srv.response.ros_is_received: %d", chassis_send_event_srv.response.ros_is_received);
    }
    else
    {
        ROS_ERROR("Failed to call service chassis_send_event_srv");
    }
}

void Chassis::PubOdomToRosOdom(Odometry odom_data)
{
    // ROS_INFO_STREAM("Odom_data to be published: " <<
    //     odom_data.pose_.orientation  << ","<< odom_data.pose_.x << "," << odom_data.pose_.y);
    ROS_odom.header.stamp = timestamp2rostime(odom_data.TimeStamp);
    ROS_odom.header.frame_id = "odom";
    ROS_odom.pose.pose.position.x = odom_data.pose_.x;
    ROS_odom.pose.pose.position.y = odom_data.pose_.y;
    ROS_odom.pose.pose.position.z = 0;
    ROS_odom.pose.pose.orientation.x = 0;
    ROS_odom.pose.pose.orientation.y = 0;
    ROS_odom.pose.pose.orientation.z = sin(odom_data.pose_.orientation / 2);
    ROS_odom.pose.pose.orientation.w = cos(odom_data.pose_.orientation / 2);
    ROS_odom.twist.twist.linear.x = odom_data.twist_.v_x;
    ROS_odom.twist.twist.linear.y = odom_data.twist_.v_y;
    ROS_odom.twist.twist.linear.z = 0;
    ROS_odom.twist.twist.angular.x = 0;
    ROS_odom.twist.twist.angular.y = 0;
    ROS_odom.twist.twist.angular.z = odom_data.twist_.w_z;
    Odom_pub.publish(ROS_odom);
}

void Chassis::PubImuToRosImu(void)
{
    uint64_t imu_stamp = ImuGyr_TimeStamp > ImuAcc_TimeStamp ? ImuGyr_TimeStamp : ImuAcc_TimeStamp;
    ros_imu.header.seq++;
    ros_imu.header.stamp = timestamp2rostime(imu_stamp);
    ros_imu.header.frame_id = "robot_imu";
    ros_imu.angular_velocity.x = (double)ImuGyrData.gyr[0] / 900.0;           //* IMU_ANGULAR_VEL_CONVERT_UINIT;
    ros_imu.angular_velocity.y = (double)ImuGyrData.gyr[1] / 900.0;           //* IMU_ANGULAR_VEL_CONVERT_UINIT;
    ros_imu.angular_velocity.z = (double)ImuGyrData.gyr[2] / 900.0;           //* IMU_ANGULAR_VEL_CONVERT_UINIT;
    ros_imu.linear_acceleration.x = (double)ImuAccData.acc[0] / 4000.0 * 9.8; // * IMU_LINEAR_VEL_CONVERT_UINIT;
    ros_imu.linear_acceleration.y = (double)ImuAccData.acc[1] / 4000.0 * 9.8; // * IMU_LINEAR_VEL_CONVERT_UINIT;
    ros_imu.linear_acceleration.z = (double)ImuAccData.acc[2] / 4000.0 * 9.8; // * IMU_LINEAR_VEL_CONVERT_UINIT;
    Imu_pub.publish(ros_imu);
    // ROS_INFO("ros_imu:angular_vel:%f  rad/s   linear_acc:%f  m/s2",
    //             ros_imu.angular_velocity.z, ros_imu.linear_acceleration.x);
}

void Chassis::TimeUpdate1000Hz(const ros::TimerEvent &event)
{
    if (Speed_update == 1)
    {
        speed_fb.car_speed = SpeedData.car_speed;
        speed_fb.car_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.turn_speed = SpeedData.turn_speed;
        speed_fb.turn_speed /= ANGULAR_SPEED_TRANS_GAIN_RADPS;
        speed_fb.l_speed = SpeedData.l_speed;
        speed_fb.l_speed /= LINE_SPEED_TRANS_GAIN_MPS; //change the units from m/h to m/s
        speed_fb.r_speed = SpeedData.r_speed;
        speed_fb.r_speed /= LINE_SPEED_TRANS_GAIN_MPS; //change the units from m/h to m/s
        speed_fb.speed_timestamp = Speed_TimeStamp;
        Speed_update = 0;
        speed_fb_pub.publish(speed_fb);
        // ROS_INFO("###chassis speed_fb, l_speed:%f m/s, r_speed:%f m/s", speed_fb.l_speed, speed_fb.r_speed);
    }

    if (Ticks_update == 1)
    {
        ticks_fb.l_ticks = TicksData.l_ticks;
        ticks_fb.r_ticks = TicksData.r_ticks;
        ticks_fb.ticks_timestamp = Ticks_TimeStamp;
        Ticks_update = 0;
        ticks_fb_pub.publish(ticks_fb);
#ifndef ODOM_BY_CHASSIS
        SensorData::Ticks tick_msg(Ticks_TimeStamp, TicksData.l_ticks, TicksData.r_ticks);
        if (robot::Chassis::m_ge_encoder.add_ticks(tick_msg))
        {
            Odometry odome = robot::Chassis::m_ge_encoder.GetOdometry();
            PubOdomToRosOdom(odome);
        }
#endif
    }

    if (ImuGyr_update == 1 && ImuAcc_update == 1)
    {
#ifndef ODOM_BY_CHASSIS
        double yaw_radps = 0;
        yaw_radps = (double)ImuGyrData.gyr[2] / 900; //* IMU_ANGULAR_VEL_CONVERT_UINIT;
        SensorData::BaseImu imu_msg(ImuAcc_TimeStamp, yaw_radps);
        if (robot::Chassis::m_ge_encoder.add_imubase(imu_msg))
        {
            Odometry odome = robot::Chassis::m_ge_encoder.GetOdometry();
            PubOdomToRosOdom(odome);
        }
#endif
        PubImuToRosImu();

        ImuGyr_update = 0;
        ImuAcc_update = 0;
    }

#ifdef ODOM_BY_CHASSIS
    static uint64_t time_pre = 0;
    if (Odom_update == 15)
    {
        Odom_update = 0;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(OdomEulerZ.euler_z / RAD_DEGREE_CONVER);

        ROS_odom.header.stamp = timestamp2rostime(Odom_TimeStamp);
        ROS_odom.header.frame_id = "odom";
        ROS_odom.pose.pose.position.x = OdomPoseXy.pos_x;
        ROS_odom.pose.pose.position.y = OdomPoseXy.pos_y;
        ROS_odom.pose.pose.position.z = 0;
        ROS_odom.pose.pose.orientation.x = getOrientationX();
        ROS_odom.pose.pose.orientation.y = getOrientationY();
        ROS_odom.pose.pose.orientation.z = getOrientationZ();
        ROS_odom.pose.pose.orientation.w = getOrientationW();
        ROS_odom.child_frame_id = "base_link";
        ROS_odom.twist.twist.linear.x = (double)SpeedData.car_speed / LINE_SPEED_TRANS_GAIN_MPS;
        ROS_odom.twist.twist.linear.y = 0;
        ROS_odom.twist.twist.linear.z = 0;
        ROS_odom.twist.twist.angular.x = (double)ImuGyrData.gyr[0] / 900.0;
        ROS_odom.twist.twist.angular.y = (double)ImuGyrData.gyr[1] / 900.0;
        ROS_odom.twist.twist.angular.z = (double)ImuGyrData.gyr[2] / 900.0; //* IMU_ANGULAR_VEL_CONVERT_UINIT;

        odom_trans.header.stamp = timestamp2rostime(Odom_TimeStamp);
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = OdomPoseXy.pos_x;
        odom_trans.transform.translation.y = OdomPoseXy.pos_y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        if ((Odom_TimeStamp - time_pre) > 100000)
        {
            static uint8_t first = 1;
            if (first)
            {
                first = 0;
            }
            else
            {
                //uint64_t timegap = (Odom_TimeStamp - time_pre);
                //printf("!!!!!!!!!! timeout !!!timegap: %lu, curtime:%lu, pretime:%lu\n", timegap, Odom_TimeStamp, time_pre);
            }
        }
        time_pre = Odom_TimeStamp;
        Odom_pub.publish(ROS_odom);
    }
#endif
    ros::spinOnce();
}

void Chassis::TimeUpdate1Hz(const ros::TimerEvent &event)
{
    bms_fb.bat_soc = get_bat_soc();
    bms_fb.bat_charging = get_bat_charging();
    bms_fb.bat_vol = get_bat_mvol();
    bms_fb.bat_current = get_bat_mcurrent();
    bms_fb.bat_temp = get_bat_temp();
    bms_fb_pub.publish(bms_fb);

    chassis_ctrl_src_fb.chassis_ctrl_cmd_src = get_ctrl_cmd_src();
    chassis_ctrl_src_fb_pub.publish(chassis_ctrl_src_fb);

    chassis_mileage_meter_fb.vehicle_meters = get_vehicle_meter();
    chassis_mileage_meter_fb_pub.publish(chassis_mileage_meter_fb);

    chassis_mode_fb.chassis_mode = get_chassis_mode(); //0: lock_mode, 1:ctrl_mode, 2:push_mode, 3:emergency mode, 4:error mode
    chassis_mode_fb_pub.publish(chassis_mode_fb);

    error_code_fb.host_error = get_err_state(Host);
    error_code_fb.central_error = get_err_state(Central);
    error_code_fb.left_motor_error = get_err_state(Motor) & 0xffff;
    error_code_fb.right_motor_error = (get_err_state(Motor) >> 16) & 0xffff;
    error_code_fb.bms_error = get_err_state(BMS);
    error_code_fb_pub.publish(error_code_fb);
    
    motor_work_mode_fb.motor_work_mode = get_chassis_work_model();
    motor_work_mode_fb_pub.publish(motor_work_mode_fb);

    ros::spinOnce();
}
} // namespace robot
#endif
