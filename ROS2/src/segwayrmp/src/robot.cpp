#include "segwayrmp/robot.h"
#include <cwchar>
#define VEL_DT              0.05  //50ms
#define RAD_DEGREE_CONVER   57.2958
#define HOST_SPEED_UINIT            3600 //
#define HOST_ANGULAR_SPEED_UINIT    2271

typedef struct{
    uint32_t typeId;
    uint8_t   dataSize;
    uint64_t *segwayTimeStamp;
    uint8_t   * updateFlag;
    void * dataPtr;
}SegwayData;

chassis_speed_data_t SpeedData;
uint64_t    Speed_TimeStamp;
uint8_t      Speed_update;

motor_ticks_t TickData;
uint64_t    Tick_TimeStamp;
uint8_t     Tick_update;

imu_gyr_original_data_t  ImuGyroData;
uint64_t    ImuGyro_TimeStamp;
uint8_t     ImuGyro_update;

imu_acc_original_data_t ImuAccData;
uint64_t    ImuAcc_TimeStamp;
uint8_t     ImuAcc_update;

odom_pos_xy_t OdomPoseXy;
odom_euler_xy_t OdomEulerXy;
odom_euler_z_t OdomEulerZ;
odom_vel_line_xy_t OdomVelLineXy;
uint64_t Odom_TimeStamp;
uint8_t OdomPoseXy_update;
uint8_t OdomEulerXy_update;
uint8_t OdomEulerZ_update;
uint8_t OdomVelLineXy_update;

static SegwayData segway_data_tbl[] = {
    {Chassis_Data_Speed,  sizeof(SpeedData), &Speed_TimeStamp, &Speed_update, &SpeedData},
    {Chassis_Data_Ticks,  sizeof(TickData), &Tick_TimeStamp, &Tick_update, &TickData},
    {Chassis_Data_Imu_Gyr,  sizeof(ImuGyroData), &ImuGyro_TimeStamp, &ImuGyro_update, &ImuGyroData},
    {Chassis_Data_Imu_Acc,  sizeof(ImuAccData), &ImuAcc_TimeStamp, &ImuAcc_update, &ImuAccData},
    {Chassis_Data_Odom_Pose_xy,  sizeof(OdomPoseXy), &Odom_TimeStamp, &OdomPoseXy_update, &OdomPoseXy},
    {Chassis_Data_Odom_Euler_xy,  sizeof(OdomEulerXy), &Odom_TimeStamp, &OdomEulerXy_update, &OdomEulerXy},
    {Chassis_Data_Odom_Euler_z,  sizeof(OdomEulerZ), &Odom_TimeStamp, &OdomEulerZ_update, &OdomEulerZ},
    {Chassis_Data_Odom_Linevel_xy,  sizeof(OdomVelLineXy), &Odom_TimeStamp, &OdomVelLineXy_update, &OdomVelLineXy}  
};

std::shared_ptr<rclcpp::Node> car_node;
rclcpp::Client<segway_msgs::srv::ChassisSendEvent>::SharedPtr event_client;

void PubData(StampedBasicFrame_ *frame)
{
    uint8_t i = 0;
    for (; i < sizeof(segway_data_tbl)/sizeof(segway_data_tbl[0]); i++)
    {
        if (frame->type_id == segway_data_tbl[i].typeId) break;
    }
    if (i < sizeof(segway_data_tbl)/sizeof(segway_data_tbl[0])){
        *(segway_data_tbl[i].segwayTimeStamp) = frame->timestamp;
        *(segway_data_tbl[i].updateFlag) =1;
        memcpy( segway_data_tbl[i].dataPtr, frame->data,  segway_data_tbl[i].dataSize);
    }
}

void EventPubData(int event_no)
{
    robot::Chassis::pub_event_callback(event_no);
}

namespace robot
{
void Chassis::pub_event_callback(int event_no)
{
    using eventServiceResponseFutrue = rclcpp::Client<segway_msgs::srv::ChassisSendEvent>::SharedFuture;
    auto event_request = std::make_shared<segway_msgs::srv::ChassisSendEvent::Request>();
    event_request->chassis_send_event_id = event_no;
    auto event_response_receive_callback = [](eventServiceResponseFutrue futrue) {
        (void)futrue;
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "event sended successfully");
    };
    auto event_future_result = event_client->async_send_request(event_request, event_response_receive_callback);
}

Chassis::Chassis(rclcpp::Node::SharedPtr nh) : node(nh)
{
    using namespace std::placeholders;
    car_node = nh;

    timestamp_data.on_new_data = PubData;
    aprctrl_datastamped_jni_register(&timestamp_data);
    event_data.event_callback = EventPubData;
    aprctrl_eventcallback_jni_register(&event_data);

    node->declare_parameter<std::string>("bins_directory", "/sdcard/firmware/");
    node->declare_parameter<std::string>("central_version", "1.01");
    node->declare_parameter<std::string>("motor_version", "1.01");

    bms_fb_pub = node->create_publisher<segway_msgs::msg::BmsFb>("/bms_fb", 1);
    chassis_ctrl_src_fb_pub = node->create_publisher<segway_msgs::msg::ChassisCtrlSrcFb>("/chassis_ctrl_src_fb", 1);
    chassis_mileage_meter_fb_pub = node->create_publisher<segway_msgs::msg::ChassisMileageMeterFb>("/chassis_mileage_meter_fb", 1);
    chassis_mode_fb_pub = node->create_publisher<segway_msgs::msg::ChassisModeFb>("/chassis_mode_fb", 1);
    error_code_fb_pub = node->create_publisher<segway_msgs::msg::ErrorCodeFb>("/error_code_fb", 1);
    motor_work_mode_fb_pub = node->create_publisher<segway_msgs::msg::MotorWorkModeFb>("/motor_work_mode_fb", 1);
    speed_fb_pub = node->create_publisher<segway_msgs::msg::SpeedFb>("/speed_fb", 1);
    ticks_fb_pub = node->create_publisher<segway_msgs::msg::TicksFb>("/ticks_fb", 1);
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 1);

    event_client = node->create_client<segway_msgs::srv::ChassisSendEvent>("event_srv");

    ros_get_charge_mos_ctrl_status_cmd_server = node->create_service<segway_msgs::srv::RosGetChargeMosCtrlStatusCmd>(
       "get_charge_mos_ctrl_status", std::bind(&Chassis::ros_get_charge_mos_ctrl_status_cmd_callback, this, _1, _2));
    ros_get_load_param_cmd_server = node->create_service<segway_msgs::srv::RosGetLoadParamCmd>(
       "get_load_param", std::bind(&Chassis::ros_get_load_param_cmd_callback, this, _1, _2));
    ros_get_sw_version_cmd_server = node->create_service<segway_msgs::srv::RosGetSwVersionCmd>(
       "get_sw_version", std::bind(&Chassis::ros_get_sw_version_cmd_callback, this, _1, _2));
    ros_get_vel_max_feedback_cmd_server = node->create_service<segway_msgs::srv::RosGetVelMaxFeedbackCmd>(
       "get_vel_max_feedback", std::bind(&Chassis::ros_get_vel_max_feedback_cmd_callback, this, _1, _2));
    ros_set_charge_mos_ctrl_cmd_server = node->create_service<segway_msgs::srv::RosSetChargeMosCtrlCmd>(
       "set_charge_mos_ctrl", std::bind(&Chassis::ros_set_charge_mos_ctrl_cmd_callback, this, _1, _2));
    ros_set_chassis_enable_cmd_server = node->create_service<segway_msgs::srv::RosSetChassisEnableCmd>(
       "set_chassis_enable", std::bind(&Chassis::ros_set_chassis_enable_cmd_callback, this, _1, _2));
    ros_set_chassis_calib_imu_cmd_server = node->create_service<segway_msgs::srv::RosSetChassisCalibImuCmd>(
       "set_chassis_calib_imu", std::bind(&Chassis::ros_set_chassis_calib_imu_cmd_callback, this, _1, _2));
    ros_set_chassis_poweroff_cmd_server = node->create_service<segway_msgs::srv::RosSetChassisPoweroffCmd>(
       "set_chassis_poweroff", std::bind(&Chassis::ros_set_chassis_poweroff_cmd_callback, this, _1, _2));
    ros_set_load_param_cmd_server = node->create_service<segway_msgs::srv::RosSetLoadParamCmd>(
       "set_load_param", std::bind(&Chassis::ros_set_load_param_cmd_callback, this, _1, _2));
    ros_set_remove_push_cmd_server = node->create_service<segway_msgs::srv::RosSetRemovePushCmd>(
       "set_remove_push", std::bind(&Chassis::ros_set_remove_push_cmd_callback, this, _1, _2));
    ros_set_vel_max_cmd_server = node->create_service<segway_msgs::srv::RosSetVelMaxCmd>(
       "set_vel_max", std::bind(&Chassis::ros_set_vel_max_cmd_callback, this, _1, _2));
    ros_get_low_power_shutdown_threshold_cmd_server = node->create_service<segway_msgs::srv::RosGetLowPowerShutdownThresholdCmd>(
       "ros_get_low_power_shutdown_threshold", std::bind(&Chassis::ros_get_low_power_shutdown_threshold_cmd_callback, this, _1, _2));
    ros_set_low_power_shutdown_threshold_cmd_server = node->create_service<segway_msgs::srv::RosSetLowPowerShutdownThresholdCmd>(
       "ros_set_low_power_shutdown_threshold", std::bind(&Chassis::ros_set_low_power_shutdown_threshold_cmd_callback, this, _1, _2));

    velocity_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&Chassis::cmd_vel_callback, this, std::placeholders::_1));



    iap_action_server = rclcpp_action::create_server<iapCmd>(
        node, 
        "iapCmd", 
        std::bind(&Chassis::handle_iapCmdGoal, this, _1, _2),
        std::bind(&Chassis::handle_iapCmdCancel, this, _1),
        std::bind(&Chassis::handle_iapCmdAccepted, this, _1));
 
    odom_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    timer_1000hz = node->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Chassis::timer_1000hz_callback, this)); 
    timer_1hz = node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Chassis::timer_1hz_callback, this));
}

void Chassis::ros_get_charge_mos_ctrl_status_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetChargeMosCtrlStatusCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosGetChargeMosCtrlStatusCmd::Response> response)
{
    int16_t ret = 0;
    if (request->ros_get_chassis_charge_ctrl_status == true){
        ret = get_charge_mos_ctrl_status();
        response->chassis_charge_ctrl_status = ret;
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "receive RosGetChargeMosCtrlStatusCmd cmd[%d], charge_ctrl_status[%d]", 
                1, ret);
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_get_chassis_charge_ctrl_status false");
    }
}
void Chassis::ros_get_load_param_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetLoadParamCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosGetLoadParamCmd::Response> response)
{
    if (request->ros_get_load_param == false) {
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_get_load_param fasle");
        return ;
    }
    response->get_load_param = get_chassis_load_state();
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "get_load_param[%d]", response->get_load_param );
}
void Chassis::ros_get_sw_version_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetSwVersionCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosGetSwVersionCmd::Response> response)
{
    if (request->ros_get_sw_version_cmd == false) {
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_get_sw_version_cmd fasle");
        return;
    }
    response->host_version = get_host_version();
    response->central_version = get_chassis_central_version();
    response->motor_version = get_chassis_motor_version();
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_get_sw_version_cmd true");
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "host_version:%d, central_version:%d, motor_version:%d", 
        response->host_version, response->central_version, response->motor_version);
}
void Chassis::ros_get_vel_max_feedback_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetVelMaxFeedbackCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosGetVelMaxFeedbackCmd::Response> response)
{
    if (request->ros_get_vel_max_fb_cmd == false) {
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_get_vel_max_fb_cmd fasle");
        return;
    }
    response->forward_max_vel_fb = get_line_forward_max_vel_fb();
    response->backward_max_vel_fb = get_line_backward_max_vel_fb();
    response->angular_max_vel_fb = get_angular_max_vel_fb();
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_get_vel_max_fb_cmd true");
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "forward_max_vel_fb:%d, backward_max_vel_fb:%d, angular_max_vel_fb:%d", 
        response->forward_max_vel_fb, response->backward_max_vel_fb, response->angular_max_vel_fb);
}
void Chassis::ros_set_charge_mos_ctrl_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChargeMosCtrlCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetChargeMosCtrlCmd::Response> response)
{
    uint8_t ret;
    if (request->ros_set_chassis_charge_ctrl == false) {
        ret = set_charge_mos_ctrl(false);
    }
    else {
        ret = set_charge_mos_ctrl(true);
    }
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_set_chassis_charge_ctrl cmd[%d]", request->ros_set_chassis_charge_ctrl);
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "chassis_set_charge_ctrl_result[%d]", ret);
    response->chassis_set_charge_ctrl_result = ret;
}
void Chassis::ros_set_chassis_enable_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChassisEnableCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetChassisEnableCmd::Response> response)
{
    uint8_t ret;
    if (request->ros_set_chassis_enable_cmd == false) {
        ret = set_enable_ctrl(0);
    }
    else {
        ret = set_enable_ctrl(1);
    }
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_set_chassis_enable_cmd cmd[%d]", request->ros_set_chassis_enable_cmd);
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "chassis_set_chassis_enable_result[%d]", ret);
    response->chassis_set_chassis_enable_result = ret;
}
void Chassis::ros_set_chassis_calib_imu_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChassisCalibImuCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetChassisCalibImuCmd::Response> response)
{
    (void)request;
    int8_t calib_ret = set_calib_gyro();
    response->chassis_calib_imu_result = calib_ret;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "response->chassis_calib_imu_result:%d[0:success; -1:fail to send cmd; -2:fail to calib; -3: overtime] ", calib_ret);
}                
void Chassis::ros_set_chassis_poweroff_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetChassisPoweroffCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetChassisPoweroffCmd::Response> response)
{
    if (request->ros_set_chassis_poweroff_cmd == false) {
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_set_chassis_poweroff_cmd fasle");
        return;
    }
    uint8_t ret = set_chassis_poweroff();
    response->chassis_set_poweroff_result = ret;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "chassis_set_poweroff_result[%d]", ret);
}
void Chassis::ros_set_load_param_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetLoadParamCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetLoadParamCmd::Response> response)
{
    int16_t value = request->ros_set_load_param;
    if (value != 0 && value != 1){
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_set_load_param[%d] out of normal range ", value);
        return;
    }
    uint8_t ret = set_chassis_load_state(value);
    response->chassis_set_load_param_result = ret;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "chassis_set_load_param_result[%d]", ret);
}
void Chassis::ros_set_remove_push_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetRemovePushCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetRemovePushCmd::Response> response)
{
    if (request->ros_set_remove_push_cmd == false) {
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_set_remove_push_cmd fasle");
        return;
    }
    uint8_t ret = set_remove_push_cmd();
    response->chassis_set_revove_push_result = ret;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "chassis_set_revove_push_result[%d]", ret);
}
void Chassis::ros_set_vel_max_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetVelMaxCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetVelMaxCmd::Response> response)
{
    double forward = request->ros_set_forward_max_vel;
    double backward = request->ros_set_backward_max_vel;
    double angular = request->ros_set_angular_max_vel;
    uint8_t ret_forw = set_line_forward_max_vel(forward);
    uint8_t ret_back = set_line_backward_max_vel(backward);
    uint8_t ret_angl = set_angular_max_vel(angular);
    response->chassis_set_max_vel_result = ret_forw | ret_back | ret_angl;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "ros_set_forward_max_vel[%lf], ros_set_backward_max_vel[%lf], ros_set_angular_max_vel[%lf]", 
        forward, backward, angular);
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "set_line_forward_max_vel result[%d], set_line_backward_max_vel result[%d], set_angular_max_vel result[%d]", 
        ret_forw, ret_back, ret_angl);
}
void Chassis::ros_get_low_power_shutdown_threshold_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosGetLowPowerShutdownThresholdCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosGetLowPowerShutdownThresholdCmd::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "request: [%d]", request);  //no used
    
    uint16_t threshold_soc = 0;
    threshold_soc = get_low_power_shutdown_threshold();
    response->chassis_get_soc_threshold_result = threshold_soc;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "response->chassis_get_soc_threshold_result:[%d], get_low_power_shutdown_threshold():[%d] ", response->chassis_get_soc_threshold_result,threshold_soc);
}
void Chassis::ros_set_low_power_shutdown_threshold_cmd_callback(const std::shared_ptr<segway_msgs::srv::RosSetLowPowerShutdownThresholdCmd::Request> request,
    std::shared_ptr<segway_msgs::srv::RosSetLowPowerShutdownThresholdCmd::Response> response)
{
    uint8_t threshold_ret = set_low_power_shutdown_threshold(request->ros_set_low_power_shutdown_threshold);
    response->chassis_set_soc_threshold_result = threshold_ret;
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "request->ros_set_low_power_shutdown_threshold: [%d]", request->ros_set_low_power_shutdown_threshold);
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "response->chassis_set_soc_threshold_result:%d[0:success; -1:fail to send cmd; -2:fail to calib; -3: overtime] ", threshold_ret);
}

void Chassis::iapCmdExecute(const std::shared_ptr<goalHandaleIapCmd> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<iapCmd::Feedback>();
    auto result = std::make_shared<iapCmd::Result>();
    int32_t iap_percent_fb;
    
    node->get_parameter("bins_directory", bins_directory);
    node->get_parameter("central_version", central_version);
    node->get_parameter("motor_version", motor_version);
    char bin_dir[100] = {0};
    char ver[100] = {0};
    switch (goal->iap_board)
    {
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "iap central board");
        bins_directory = bins_directory + "/central.bin";
        bins_directory.copy(bin_dir, bins_directory.length(), 0);
        central_version.copy(ver, central_version.length(), 0);
        IapSingerBoard(bin_dir, (char*)"central", ver);
        break;
    case 2:
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "iap motor board");
        bins_directory = bins_directory + "/motor.bin";
        bins_directory.copy(bin_dir, bins_directory.length(), 0);
        motor_version.copy(ver, motor_version.length(), 0);
        IapSingerBoard(bin_dir, (char*)"motor", ver);
        break;
    default:
        RCLCPP_ERROR(rclcpp::get_logger("SmartCar"), 
        "iap_board value error, out of [1 2]", goal->iap_board);
        result->set__iap_result(5);
        goal_handle->canceled(result);
        return;
    }

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling()) {
            result->set__iap_result(5);
            goal_handle->canceled(result);
            RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "Goal canceled");
            return;
        }
        iap_percent_fb = getIapTotalProgress();
        feedback->set__iap_percent(iap_percent_fb);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "iap percent: %d", iap_percent_fb);
        loop_rate.sleep();
        if (iap_percent_fb == 100) {
            RCLCPP_INFO_ONCE(rclcpp::get_logger("SmartCar"), "iap successful");
            break;
        } else if (iap_percent_fb < 0) {
            RCLCPP_INFO_ONCE(rclcpp::get_logger("SmartCar"), "iap failed");
            break;
        }
    }
    
    if (rclcpp::ok()) {
        if (iap_percent_fb == 100) {
            result->set__iap_result(3);
            goal_handle->succeed(result);
            RCLCPP_INFO(rclcpp::get_logger("SmartCar"), "goal succeeded, iap succeeded");
        } else {
            result->set__iap_result(getHostIapResult());
            result->set__error_code(getHostIapErrorCode());
            goal_handle->succeed(result);
            RCLCPP_ERROR(rclcpp::get_logger("SmartCar"), "goal failed, iap failed");
        }
    }
}

void Chassis::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    double angular_vel = msg->angular.z;
    double linear_vel  = msg->linear.x;
    angular_vel = (angular_vel > 3.0 ? 3.0 : (angular_vel < -3.0 ? -3.0 : angular_vel));
    linear_vel = (linear_vel > 3.0 ? 3.0 : (linear_vel < -1.0 ? -1.0 : linear_vel));

    set_cmd_vel(linear_vel, angular_vel);
    // printf("vl:%lf, va:%lf\n", linear_vel, angular_vel);
}

void Chassis::timer_1hz_callback(void)
{
    bms_fb.bat_soc = get_bat_soc();
    bms_fb.bat_charging = get_bat_charging();
    bms_fb.bat_vol = get_bat_mvol();
    bms_fb.bat_current = get_bat_mcurrent();
    bms_fb.bat_temp = get_bat_temp();
    bms_fb_pub->publish(bms_fb);

    chassis_ctrl_src_fb.chassis_ctrl_cmd_src = get_ctrl_cmd_src();
    chassis_ctrl_src_fb_pub->publish(chassis_ctrl_src_fb);

    chassis_mileage_meter_fb.vehicle_meters = get_vehicle_meter();
    chassis_mileage_meter_fb_pub->publish(chassis_mileage_meter_fb);

    chassis_mode_fb.chassis_mode = get_chassis_mode();//0-lock_mode; 1-ctrl_mode; 2-push_mode; 3-emergency_mode; 4-error_mode
    chassis_mode_fb_pub->publish(chassis_mode_fb);

    error_code_fb.host_error = get_err_state(Host);
    error_code_fb.central_error = get_err_state(Central);
    error_code_fb.left_motor_error = get_err_state(Motor) & 0xffff;
    error_code_fb.right_motor_error = (get_err_state(Motor) >> 16) & 0xffff;
    error_code_fb.bms_error = get_err_state(BMS);
    error_code_fb_pub->publish(error_code_fb);

    motor_work_mode_fb.motor_work_mode = get_chassis_work_model();
    motor_work_mode_fb_pub->publish(motor_work_mode_fb);
}

void Chassis::timer_1000hz_callback(void)
{
    speed_pub_callback();
    ticks_pub_callback();
    imu_pub_callback();
    pub_odom_callback();
}

void Chassis::speed_pub_callback(void)
{
    if (Speed_update == 1) {
        Speed_update = 0;
        speed_fb.car_speed = SpeedData.car_speed;
        speed_fb.car_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.turn_speed = SpeedData.turn_speed;
        speed_fb.turn_speed /= ANGULAR_SPEED_TRANS_GAIN_RADPS;
        speed_fb.l_speed = SpeedData.l_speed;
        speed_fb.l_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.r_speed = SpeedData.r_speed;
        speed_fb.r_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.speed_timestamp = Speed_TimeStamp;
        speed_fb_pub->publish(speed_fb);
    }
}

void Chassis::ticks_pub_callback(void)
{
    if (Tick_update == 1) {
        Tick_update = 0;
        ticks_fb.l_ticks = TickData.l_ticks;
        ticks_fb.r_ticks = TickData.r_ticks;
        ticks_fb.ticks_timestamp = Tick_TimeStamp;
        ticks_fb_pub->publish(ticks_fb);
    }
}

void Chassis::imu_pub_callback(void)
{
    if (ImuGyro_update == 1 && ImuAcc_update == 1){
        ImuGyro_update = 0;
        ImuAcc_update = 0;
        imu_fb.header.stamp = node->now();
        imu_fb.header.frame_id = "base_link";
        imu_fb.angular_velocity.x = (double)ImuGyroData.gyr[0] / 900.0;
        imu_fb.angular_velocity.y = (double)ImuGyroData.gyr[1] / 900.0;
        imu_fb.angular_velocity.z = (double)ImuGyroData.gyr[2] / 900.0;
        imu_fb.linear_acceleration.x = (double)ImuAccData.acc[0] / 4000.0 * 9.8;
        imu_fb.linear_acceleration.y = (double)ImuAccData.acc[1] / 4000.0 * 9.8;
        imu_fb.linear_acceleration.z = (double)ImuAccData.acc[2] / 4000.0 * 9.8;
        imu_pub->publish(imu_fb);
    }
}

void Chassis::pub_odom_callback(void)
{    
    if ((OdomPoseXy_update & OdomEulerXy_update & OdomEulerZ_update & OdomVelLineXy_update) == 1)  {
        OdomPoseXy_update = 0, OdomEulerXy_update = 0, OdomEulerZ_update = 0, OdomVelLineXy_update = 0;

        odom_quat.setRPY(0, 0, OdomEulerZ.euler_z / RAD_DEGREE_CONVER);
        odom_trans.header.stamp = node->now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = OdomPoseXy.pos_x;
        odom_trans.transform.translation.y = OdomPoseXy.pos_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.x = odom_quat.x();
        odom_trans.transform.rotation.y = odom_quat.y();
        odom_trans.transform.rotation.z = odom_quat.z();
        odom_trans.transform.rotation.w = odom_quat.w();
        odom_broadcaster->sendTransform(odom_trans);

        odom_fb.header.stamp = node->now();
        odom_fb.header.frame_id = "odom";
        odom_fb.pose.pose.position.x = OdomPoseXy.pos_x;
        odom_fb.pose.pose.position.y = OdomPoseXy.pos_y;
        odom_fb.pose.pose.position.z = 0.0;
        odom_fb.pose.pose.orientation.x = odom_quat.x();
        odom_fb.pose.pose.orientation.y = odom_quat.y();
        odom_fb.pose.pose.orientation.z = odom_quat.z();
        odom_fb.pose.pose.orientation.w = odom_quat.w();

        odom_fb.child_frame_id = "base_link";
        odom_fb.twist.twist.linear.x = (double)SpeedData.car_speed / LINE_SPEED_TRANS_GAIN_MPS;
        odom_fb.twist.twist.linear.y = 0;
        odom_fb.twist.twist.angular.z = (double)SpeedData.turn_speed / ANGULAR_SPEED_TRANS_GAIN_RADPS;;
        odom_pub->publish(odom_fb);
    }
}

}
