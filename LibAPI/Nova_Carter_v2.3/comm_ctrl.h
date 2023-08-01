/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2023,Segway-Ninebot,China,beijing
  *                            All Rights Reserved
  *                              
  *    
  * FileName   : comm_ctrl.h   
  * Version    : v1.0
  * Author     : SegwayRobotics Team
  * Date       : 2023-08-01 
  * Description:  
  *   1. Dynamic link library interface.
  *
  * License:
  *     Copyright [2023] [Segway Robotics]
  *
  *		Licensed under the Apache License, Version 2.0 (the "License");
  *		you may not use this file except in compliance with the License.
  *		You may obtain a copy of the License at
  *
  *			http://www.apache.org/licenses/LICENSE-2.0
  *
  *		Unless required by applicable law or agreed to in writing, software
  *		distributed under the License is distributed on an "AS IS" BASIS,
  *		WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  *		See the License for the specific language governing permissions and
  *		limitations under the License.
  *		
  * https://robotics.segway.com
  * http://www.segwayrobotics.com
  * https://www.ninebot.com
  *
  ******************************************************************************
 */


#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMM_CTRL_H
#define COMM_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include "comm_ctrl_navigation.h"


// Map table transmission value gain coefficient;
#define ERROR_LINE_LIMIT_VEL_VALUE		(5 * LINE_SPEED_TRANS_GAIN_MPS)
#define ERROR_ANGULAR_LIMIT_VEL_VALUE	(5 * ANGULAR_SPEED_TRANS_GAIN_RADPS)
#define ACC_VALUE_GAIN	100


#define		RMP_HOST_ERR_BIT_MCU_HEART_BEAT		0 //The heartbeat with the chassis was interrupted
#define		RMP_HOST_ERR_BIT_SERIAL_PORT_LOST	1 //Serial port module unplugged

void AprGxJniEventRegister(int32_t (*callback)(int32_t));

//-------------------Timestamp------------------------

//--------------------GX--API------------------
int init_control(void);
void exit_control(void);

int16_t get_forward_speed_limit(void);
int16_t get_backward_speed_limit(void);
int16_t get_angular_speed_limit(void);
uint16_t get_Stop_Bottom_Status(void);
uint8_t set_calib_chassis_imu_gyro(void);//Reserve for later versions
uint16_t get_Chassis_Imu_Calib_Result(void);   //Reserve for later versions
void clear_Chassis_Imu_Calib_Result(void);//Reserve for later versions
uint8_t  set_calib_chassis_current(void);
uint16_t get_chassis_current_calib_result(void);
void     clear_chassis_current_calib_result(void);


//-----------------GX  IAP--------------------
void IapAllBoard(const char **file_paths, char **versions);
int32_t IapSingerBoard(char * path,char * boradname,char* version);
//int32_t getIapTotalProgress(void);
//------------------GX Host-------------------

char* GetGxHardVersion(void);
void TracePrint_Switch(bool status);  //true  or false
void SetNavigationStatus(uint8_t status);
void setSaveRecordCmd(void);    //开始保存log接口　　　　　ps:上层调用
void clearRecordCmd(void);      //清除保存log命令　　　　　ps:上层不需要调�?so每次保存完log后会自动调用
uint8_t getSaveRecordCmd(void); //获取是否开始保存log　　　ps:so自动调用,上层不需要调�?
uint8_t GetNavigationStatus(void);


void set_gx_route_bat_voltage(uint16_t voltage);


int16_t get_cmd_speed_forward(void);
int16_t get_fbk_speed_forward(void);
int16_t get_encode_fbk_speed_forward(void);
int16_t get_cmd_w_forward(void);
int16_t get_fbk_w_forward(void);
int16_t get_rec_cmd_vx(void);
int16_t get_rec_cmd_w(void);
int16_t get_pid_target_vx(void);
int16_t get_pid_target_w(void);
int16_t get_encode_speed_L(void);
int16_t get_encode_speed_R(void);

int16_t get_heart_beat_data(board_name_e board_type);

void set_host_error(uint8_t offset, uint8_t err_bool_value);
uint32_t get_host_error(void);
uint32_t get_host_speed_cmd_cnt(void);
uint32_t get_chassis_reply_speed_cmd_cnt(void);
int32_t get_encoder_l_ticks(void);
int32_t get_encoder_r_ticks(void);
char * get_smart_car_serial(void);
uint8_t get_comu_interface_serial0_can1();
void showHostBuildTime(void);
void showChassisCentralBuildTime(void);

/* 暂时不开放此接口，由于map表只能发送整数，故此接口待修改：乘以10或�?00下发，电机板收到命令数据后再恢复数据 */
uint8_t  set_cmd_acc(double linear_x_acc, double linear_x_brake_acc, double angular_z_acc);//Set the linear and angular accelerated speed: m/s2 ; rad/s2.

uint8_t  set_chassis_no_load(void);
uint8_t  set_chassis_full_load(void);

void    ackWriteCentralMapInit();
uint8_t ctrlMapAckWrite(    uint8_t dest_id,  uint8_t map_start_index, uint32_t mem_len);

void set_chassis_charge_status(int16_t chargestatus);
uint32_t get_chassis_central_Err_Status(void);
uint16_t get_chassis_l_motor_Err_Status(void);
uint16_t get_chassis_r_motor_Err_Status(void);
uint32_t get_chassis_bms_Err_Status(void);
char * show_host_version(void);


int16_t get_charge_mos_ctrl_cmd(void);// Get chassis charge_ctrl mos cmd; 1:turn on; 0: turn off

#endif

#ifdef __cplusplus
}
#endif
// clang-format on
