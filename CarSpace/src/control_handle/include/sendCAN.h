#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "controlcan.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <pthread.h>
#include <stdlib.h>  /*标准函数库定义*/
#include <unistd.h>  /*Unix 标准函数定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <string.h>
#include <termio.h>

#define wheel_ID0          0x0000060F
#define wheel_ID1          0x0000060D
#define wheel_ID2          0x00000601
#define wheel_ID3          0x00000602

// 轮子编号
// BYTE wheel_ID[4]        ={(BYTE)0x0000060F,(BYTE)0x0000060D,(BYTE)0x00000601,(BYTE)0x00000602};
unsigned int wheel_ID[4]        ={wheel_ID0, wheel_ID1, wheel_ID2, wheel_ID3};
// 使能
UINT enable_cmd[8]      ={0x2B,0x40,0x60,0x00, 0x0F,0x00,0x00,0x00};
// 去使能
UINT unenable_cmd[8]    ={0x2B,0x40,0x60,0x00, 0x00,0x00,0x00,0x00};
// 开始CAN通信
UINT can_cmd[2]         ={0x01,0x00};
// 终止CAN通信
UINT exitcan_cmd[2]     ={0x02,0x00};
// 速度模式
UINT startspeed_cmd[8]  ={0x2F,0x60,0x60,0x00, 0x30,0x00,0x00,0x00};
// 设置速度初值
UINT setspeed_cmd[8]    ={0x23,0xff,0x60,0x00, 0x00,0x00,0x00,0x00};

extern VCI_BOARD_INFO pInfo; // 用来获取设备信息。
// extern int count ;        // 数据列表中，用来存储列表序号。
extern VCI_BOARD_INFO pInfo1[50];
extern int num ;
extern int send_frame_length ;
extern int ind_receive; // 通道号
extern int ind_send ;
extern VCI_CAN_OBJ rec[3000];

// 发现CAN设备并打印设备信息
void find_CAN_device();
// 打开CAN设备并打印设备信息
void open_CAN_device();

// 设备初始参数设置并打开
void set_CAN_para(VCI_INIT_CONFIG config);

// 发送1帧数据，含有8个字节的数据指令
void send_CAN_data(VCI_CAN_OBJ *send);

// 接收数据
void *receive_CAN_func(void *);

unsigned char first_low8bits_from_int32(int int_32);
unsigned char second_low8bits_from_int32(int int_32);
unsigned char third_low8bits_from_int32(int int_32);
unsigned char fourth_low8bits_from_int32(int int_32);

// 全部退出CAN操作模式
// extern VCI_CAN_OBJ CAN_out_cmd[1];
// void exit_can();
void exit_can(PVCI_CAN_OBJ CAN_out_cmd);
    

// 全部进入CAN操作模式
// extern VCI_CAN_OBJ CAN_operation_cmd[1];
// void start_can_operation();
void start_can_operation(PVCI_CAN_OBJ CAN_operation_cmd);

// 四个电机先去使能
// extern VCI_CAN_OBJ motor_unenable_cmd[4];
// void unenabled_motor();
void unenabled_motor(PVCI_CAN_OBJ motor_unenable_cmd);
// void unenable_motor();
void unenable_motor(PVCI_CAN_OBJ motor_unenable_cmd);

// 四个电机使能
// extern VCI_CAN_OBJ motor_enable_cmd[4];
// void enable_motor();
void enable_motor(PVCI_CAN_OBJ motor_enable_cmd);

// 四个电机进入速度模式
// extern VCI_CAN_OBJ speed_mode_cmd[4];
// void start_velocity();
void start_velocity(PVCI_CAN_OBJ speed_mode_cmd);

// 四个电机的转速初始化
// void init_speed(VCI_CAN_OBJ speed_cmd[]);
void init_speed(PVCI_CAN_OBJ speed_cmd);

// 输入四个轮子的转速，转化为标准的speed_cmd格式
// void get4WheelSpeedCmd(int speed_count_1, int speed_count_2, int speed_count_3, int speed_count_4,
//                       VCI_CAN_OBJ speed_cmd[]);
void get4WheelSpeedCmd(int speed_count_1, int speed_count_2, int speed_count_3, int speed_count_4,
                      PVCI_CAN_OBJ speed_cmd);