#include "segwayrmp/robot.h"
#include <termio.h>
#include <stdio.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#define PRINTHELP		  'h'
#define ADDLINEVEL		  'w'
#define DECLINEVEL		  's'
#define ADDANGULARVEL	  'a'
#define DECANGULARVEL	  'd'
#define PRINTCURVEL		  'f'
#define VELRESETZERO	  'g'
#define ENABLECMD         'e'
#define CHASSISPAUSE      'q'
// #define CLEARSCRAM        'c'
// #define GETERROR          'x'
#define IAPCENTRAL        'v'
#define IAPMOTOR          'b'
#define PRINTERRSW        'r'

using iapCmd = segway_msgs::action::RosSetIapCmd;
using goalHandleIapCmd = rclcpp_action::ClientGoalHandle<iapCmd>;

std::shared_ptr<rclcpp::Node> drive_node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
rclcpp::Subscription<segway_msgs::msg::ErrorCodeFb>::SharedPtr error_sub;

rclcpp::Client<segway_msgs::srv::RosSetChassisEnableCmd>::SharedPtr enable_client;
// rclcpp::Client<segway_msgs::srv::ClearChassisScramStatusCmd>::SharedPtr clear_scram_client;
// rclcpp::Client<segway_msgs::srv::GetGxErrorCmd>::SharedPtr get_err_client;
rclcpp::Service<segway_msgs::srv::ChassisSendEvent>::SharedPtr event_server;

rclcpp_action::Client<iapCmd>::SharedPtr iap_client;
uint8_t print_error = 0;

char const* print_help() {
    char const* printHelp = 
        "\t h : Displays the required keys and their meaning\n"
        "\t w : Increase forward speed by 0.1m/s\n"
        "\t s : Decrease forward speed by 0.1m/s\n"
        "\t a : Increase the angular velocity by 0.1rad/s\n"
        "\t d : Decrease the angular velocity by 0.1rad/s\n"
        "\t f : Displays current speed Settings\n"
        "\t g : Speed reset to zero\n"
        "\t e : Chassis enable switch\n"
        "\t q : Running pause. Click 'q'key again to resume running by the previous speed. W/S/A/D keys can also restore chassis running\n"
        "\t v : Iap the central board, please put the bin file in /sdcard/firmware/\n"
        "\t b : Iap the motor board, please put the bin file in /sdcard/firmware/\n"
        "\t r : Open or close the switch: printing error code\n"
        "\t others : Do nothing\n";
    return printHelp;
}

void changemode(int dir)
{
  static struct termios oldt, newt;
 
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;  //一组fd集合
 
  tv.tv_sec = 0;
  tv.tv_usec = 0;   //时间为0，非阻塞
 
  FD_ZERO(&rdfs);   //fd集合清空
  FD_SET (STDIN_FILENO, &rdfs); //增加新的fd
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);   //非阻塞检测fd为0的tty读取状态，rdfs特定位置1
  return FD_ISSET(STDIN_FILENO, &rdfs);     //判断指定的标准输入fd是否在rdfs集合中
}

static int scanKeyboard()
{
    int input_char = 0;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);  //非规范模式：每次返回单个字符
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;    //读取的最小字节数
    tcsetattr(0, TCSANOW, &new_settings);

    changemode(1);
    if (kbhit()) {
        input_char = getchar();
        printf("\n");
    }
    changemode(0);

    tcsetattr(0, TCSANOW, &stored_settings);
    return input_char;
}

char get_keyboard()
{
    char keyvalue = scanKeyboard();
    return keyvalue;
}

void goal_response_callback(std::shared_future<goalHandleIapCmd::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "Goal accepted by server, waiting for result");
  }
}

void feedback_callback(
  goalHandleIapCmd::SharedPtr,
  const std::shared_ptr<const iapCmd::Feedback> feedback)
{
  RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "IAP process percentage：[%d]", feedback->iap_percent);
}

void result_callback(const goalHandleIapCmd::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("drive_segway_sample"), "Unknown result code");
      return;
  }
  if (result.result->iap_result == 3) {
    RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap success!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap fail!, error_code[%#x]", result.result->error_code);
  }
}

void drive_chassis_test()
{
    static uint16_t set_enable_cmd = 1;
    uint8_t enable_switch = 0;
    static double set_line_speed; 
    static double set_angular_speed;
    static double send_line_speed;
    static double send_angular_speed;
    static uint8_t chassis_pause = 0;
    // uint8_t clear_scram_flag = 0;
    // uint8_t get_err_flag = 0;
    uint8_t iap_flag = 0;

    auto enable_request = std::make_shared<segway_msgs::srv::RosSetChassisEnableCmd::Request>();
    // auto clear_scram_requst = std::make_shared<segway_msgs::srv::ClearChassisScramStatusCmd::Request>();
    // auto get_err_request = std::make_shared<segway_msgs::srv::GetGxErrorCmd::Request>();

    char keyvalue = get_keyboard();
    switch (keyvalue)
    {
    case PRINTHELP:
        printf("%s\n", print_help());
        break;
    case ADDLINEVEL		:
        set_line_speed += 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case DECLINEVEL		:
        set_line_speed -= 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case ADDANGULARVEL	:
        set_angular_speed += 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case DECANGULARVEL	:
        set_angular_speed -= 0.1;
        send_line_speed = set_line_speed;
        send_angular_speed = set_angular_speed;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case PRINTCURVEL	:
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case VELRESETZERO	:
        set_line_speed = 0; 
        set_angular_speed = 0;
        send_line_speed = 0;
        send_angular_speed = 0;
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;
    case ENABLECMD      : 
        enable_switch = 1;
        enable_request->ros_set_chassis_enable_cmd = set_enable_cmd;
        ++set_enable_cmd;        
        set_enable_cmd %= 2;
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), 
                    "enable chassis switch[%d]", enable_request->ros_set_chassis_enable_cmd);
        printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        break;  
    case CHASSISPAUSE   :
        ++chassis_pause;
        chassis_pause %= 2;
        if (chassis_pause) {
            send_line_speed = 0;
            send_angular_speed = 0;
            printf("Stop the chassis temporarily\n");
        } else {
            send_line_speed = set_line_speed;
            send_angular_speed = set_angular_speed;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
        }
        break;   
    // case CLEARSCRAM :
    //     // clear_scram_flag = 1;
    //     break;
    // case GETERROR:
    //     // get_err_flag = 1;
    //     break;
    case IAPCENTRAL:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap chassis");
        iap_flag = 1;
        break;
    case IAPMOTOR:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap route");
        iap_flag = 2;
        break;
    case PRINTERRSW:
        print_error = ~print_error;
        break;
    default:
        break;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = send_line_speed;
    cmd_vel.angular.z = send_angular_speed;
    velocity_pub->publish(cmd_vel);

    if (enable_switch){
        using enableServiceResponseFutrue = rclcpp::Client<segway_msgs::srv::RosSetChassisEnableCmd>::SharedFuture;
        auto enable_response_receive_callback = [](enableServiceResponseFutrue futrue) {
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), 
            "event sended successfully, result:%d", futrue.get()->chassis_set_chassis_enable_result);
        };
        auto enable_future_result = enable_client->async_send_request(enable_request, enable_response_receive_callback);
    }

    // if (clear_scram_flag) {
    //     using clearServiceResponseFuture = rclcpp::Client<segway_msgs::srv::ClearChassisScramStatusCmd>::SharedFuture;
    //     auto clear_response_receive_callback = [](clearServiceResponseFuture future) {
    //         RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), 
    //         "clear scram status successfully, result:%d", future.get()->clear_scram_result);
    //     };
    //     auto clear_future_result = clear_scram_client->async_send_request(clear_scram_requst, clear_response_receive_callback);
    // }

    // if (get_err_flag) {
    //     using errorServiceResponseFuture = rclcpp::Client<segway_msgs::srv::GetGxErrorCmd>::SharedFuture;
    //     auto error_response_receive_callback = [](errorServiceResponseFuture future) {
    //        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), 
    //             "chassis_error_code[%d], route_error_code[%d], connect_error_code[%d]", 
    //         future.get()->chassis_error_code, 
    //         future.get()->route_error_code,
    //         future.get()->connect_error_code);
    //     };
    //     auto error_future_result = get_err_client->async_send_request(get_err_request, error_response_receive_callback);
    // }

    if (iap_flag & 3) {
        auto goal_msg = iapCmd::Goal();
        goal_msg.iap_board = iap_flag;
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "sending goal, iap cmd goal");
        auto send_goal_options = rclcpp_action::Client<iapCmd>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&goal_response_callback, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&feedback_callback, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&result_callback, std::placeholders::_1);
        iap_client->async_send_goal(goal_msg, send_goal_options);
    }
}

void get_error_code_callback(const segway_msgs::msg::ErrorCodeFb::SharedPtr msg)
{
    if (print_error != 0) {
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "host_error[%#x], central_error[%#x], \
        left_motor_error[%#x], right_motor_error[%#x], bms_err[%#x]", 
        msg->host_error, msg->central_error, msg->left_motor_error, msg->right_motor_error, msg->bms_error);
    }
}

void get_chassis_event_callback(const std::shared_ptr<segway_msgs::srv::ChassisSendEvent::Request> request,
    std::shared_ptr<segway_msgs::srv::ChassisSendEvent::Response> response)
{
    (void)response;
    switch (request->chassis_send_event_id)
    {
    case OnEmergeStopEvent:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "CHASSIS EVENT: The chassis emergency stop button is triggered");
        break;
    case OutEmergeStopEvent:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "CHASSIS EVENT: OThe chassis emergency stop button recover");
        break;
    case PadPowerOffEvent:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "CHASSIS EVENT: The chassis will power off");
        break;
    case OnLockedRotorProtectEvent:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "CHASSIS EVENT: the chassis motor locked-rotor");
        break;
    case OutLockedRotorProtectEvent:
        RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "CHASSIS EVENT: the chassis motor no longer locked-rotor");
        break;    
    default:
        break;
    }    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    drive_node = rclcpp::Node::make_shared("drive_segway_sample");

    velocity_pub = drive_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    error_sub = drive_node->create_subscription<segway_msgs::msg::ErrorCodeFb>(
        "error_code_fb", 1, std::bind(&get_error_code_callback, std::placeholders::_1));

    enable_client = drive_node->create_client<segway_msgs::srv::RosSetChassisEnableCmd>("set_chassis_enable");
    // clear_scram_client = drive_node->create_client<segway_msgs::srv::ClearChassisScramStatusCmd>("clear_scram_status_srv");
    // get_err_client = drive_node->create_client<segway_msgs::srv::GetGxErrorCmd>("get_error_srv");
    event_server = drive_node->create_service<segway_msgs::srv::ChassisSendEvent>(
        "event_srv", std::bind(&get_chassis_event_callback, std::placeholders::_1, std::placeholders::_2));

    iap_client = rclcpp_action::create_client<iapCmd>(drive_node, "iapCmd");
    
    rclcpp::TimerBase::SharedPtr timer_100hz = 
        drive_node->create_wall_timer(std::chrono::milliseconds(10), &drive_chassis_test);

    printf("%s\n", print_help());
    
    rclcpp::spin(drive_node);
    rclcpp::shutdown();
    return 0;
}