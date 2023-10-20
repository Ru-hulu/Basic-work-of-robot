#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
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
#include <signal.h>
#include <deque>
#include <yaml-cpp/yaml.h>
#include<controlcan.h>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
using namespace std;

YAML::Node config = YAML::LoadFile("/home/r/Mysoftware/CarSpace/src/control_handle/config.yaml");
const int maxCapacity = config["maxCapacity"].as<int>();
const double maxTimeDifference = config["maxTimeDifference"].as<double>();
const int speedUp_step = config["speedUp_step"].as<int>();
const int unenable_motor_usleep = config["unenable_motor_usleep"].as<int>();
const int sigint_handler_usleep = config["sigint_handler_usleep"].as<int>();
const int send_CAN_data_usleep = config["send_CAN_data_usleep"].as<int>();
const int CAN_operation_out_cmd_usleep = config["CAN_operation_out_cmd_usleep"].as<int>();
const int for_send_CAN_data_usleep = config["send_CAN_data_usleep"].as<int>();
const int for_CAN_operation_cmd_usleep = config["CAN_operation_out_cmd_usleep"].as<int>();
const int for_loop_usleep = config["for_loop_usleep"].as<int>();
const int pause_usleep = config["pause_usleep"].as<int>();
const int end_usleep = config["sigint_handler_usleep"].as<int>();
VCI_BOARD_INFO pInfo; // 用来获取设备信息。
int count = 0;        // 数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1[50];
int num = 0;
int send_frame_length = 8;
int ind_receive = 0; // 通道号
int ind_send = 0;
VCI_CAN_OBJ rec[3000];
float vvvL;
float vvvR;
float rrr=0.85;
bool newvvv = false;
unsigned char first_low8bits_from_int32(int int_32)
{
    // 此函数获取int_32的最低8bits
    unsigned char first_low8bits;
    first_low8bits = (unsigned char)int_32;
    return first_low8bits;
    // 以下语句解决linux系统不能输出字符的问题
    // printf("%d\n",low8data);
    // printf("%d\n",high8data);
    // cout<<(short)low8data<<endl;
    // cout<<(short)high8data<<endl;
}
unsigned char second_low8bits_from_int32(int int_32)
{
    // 此函数获取int_32的从低到高的第2个8bits
    unsigned char second_low8bits;
    second_low8bits = (unsigned char)(int_32 >> 8);
    return second_low8bits;
}
unsigned char third_low8bits_from_int32(int int_32)
{
    // 此函数获取int_32的从低到高的第3个8bits
    unsigned char third_low8bits;
    third_low8bits = (unsigned char)(int_32 >> 16);
    return third_low8bits;
}
unsigned char fourth_low8bits_from_int32(int int_32)
{
    // 此函数获取int_32的从低到高的第4个8bits
    unsigned char third_low8bits;
    third_low8bits = (unsigned char)(int_32 >> 24);
    return third_low8bits;
}
// 发现CAN设备并打印设备信息
void find_CAN_device()
{
    num = VCI_FindUsbDevice2(pInfo1);
    printf(">>USBCAN DEVICE NUM:");
    printf("%d", num);
    printf(" PCS");
    printf("\n");

    for (int i = 0; i < num; i++)
    {
        printf("Device:");
        printf("%d", i);
        printf("\n");
        printf(">>Get VCI_ReadBoardInfo success!\n");

        printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
        printf("%c", pInfo1[i].str_Serial_Num[1]);
        printf("%c", pInfo1[i].str_Serial_Num[2]);
        printf("%c", pInfo1[i].str_Serial_Num[3]);
        printf("%c", pInfo1[i].str_Serial_Num[4]);
        printf("%c", pInfo1[i].str_Serial_Num[5]);
        printf("%c", pInfo1[i].str_Serial_Num[6]);
        printf("%c", pInfo1[i].str_Serial_Num[7]);
        printf("%c", pInfo1[i].str_Serial_Num[8]);
        printf("%c", pInfo1[i].str_Serial_Num[9]);
        printf("%c", pInfo1[i].str_Serial_Num[10]);
        printf("%c", pInfo1[i].str_Serial_Num[11]);
        printf("%c", pInfo1[i].str_Serial_Num[12]);
        printf("%c", pInfo1[i].str_Serial_Num[13]);
        printf("%c", pInfo1[i].str_Serial_Num[14]);
        printf("%c", pInfo1[i].str_Serial_Num[15]);
        printf("%c", pInfo1[i].str_Serial_Num[16]);
        printf("%c", pInfo1[i].str_Serial_Num[17]);
        printf("%c", pInfo1[i].str_Serial_Num[18]);
        printf("%c", pInfo1[i].str_Serial_Num[19]);
        printf("\n");

        printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
        printf("%c", pInfo1[i].str_hw_Type[1]);
        printf("%c", pInfo1[i].str_hw_Type[2]);
        printf("%c", pInfo1[i].str_hw_Type[3]);
        printf("%c", pInfo1[i].str_hw_Type[4]);
        printf("%c", pInfo1[i].str_hw_Type[5]);
        printf("%c", pInfo1[i].str_hw_Type[6]);
        printf("%c", pInfo1[i].str_hw_Type[7]);
        printf("%c", pInfo1[i].str_hw_Type[8]);
        printf("%c", pInfo1[i].str_hw_Type[9]);
        printf("\n");

        printf(">>Firmware Version:V");
        printf("%x", (pInfo1[i].fw_Version & 0xF00) >> 8);
        printf(".");
        printf("%x", (pInfo1[i].fw_Version & 0xF0) >> 4);
        printf("%x", pInfo1[i].fw_Version & 0xF);
        printf("\n");
    }
    printf(">>\n");
    printf(">>\n");
    printf(">>\n");
}

// 打开CAN设备并打印设备信息
void open_CAN_device()
{
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1) // 打开设备
    {
        printf(">>open deivce success!\n"); // 打开设备成功
    }
    else
    {
        printf(">>open deivce error!\n");
        exit(1);
    }
    if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) // 读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
        printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
        printf("%c", pInfo.str_Serial_Num[1]);
        printf("%c", pInfo.str_Serial_Num[2]);
        printf("%c", pInfo.str_Serial_Num[3]);
        printf("%c", pInfo.str_Serial_Num[4]);
        printf("%c", pInfo.str_Serial_Num[5]);
        printf("%c", pInfo.str_Serial_Num[6]);
        printf("%c", pInfo.str_Serial_Num[7]);
        printf("%c", pInfo.str_Serial_Num[8]);
        printf("%c", pInfo.str_Serial_Num[9]);
        printf("%c", pInfo.str_Serial_Num[10]);
        printf("%c", pInfo.str_Serial_Num[11]);
        printf("%c", pInfo.str_Serial_Num[12]);
        printf("%c", pInfo.str_Serial_Num[13]);
        printf("%c", pInfo.str_Serial_Num[14]);
        printf("%c", pInfo.str_Serial_Num[15]);
        printf("%c", pInfo.str_Serial_Num[16]);
        printf("%c", pInfo.str_Serial_Num[17]);
        printf("%c", pInfo.str_Serial_Num[18]);
        printf("%c", pInfo.str_Serial_Num[19]);
        printf("\n");

        printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
        printf("%c", pInfo.str_hw_Type[1]);
        printf("%c", pInfo.str_hw_Type[2]);
        printf("%c", pInfo.str_hw_Type[3]);
        printf("%c", pInfo.str_hw_Type[4]);
        printf("%c", pInfo.str_hw_Type[5]);
        printf("%c", pInfo.str_hw_Type[6]);
        printf("%c", pInfo.str_hw_Type[7]);
        printf("%c", pInfo.str_hw_Type[8]);
        printf("%c", pInfo.str_hw_Type[9]);
        printf("\n");

        printf(">>Firmware Version:V");
        printf("%x", (pInfo.fw_Version & 0xF00) >> 8);
        printf(".");
        printf("%x", (pInfo.fw_Version & 0xF0) >> 4);
        printf("%x", pInfo.fw_Version & 0xF);
        printf("\n");
    }
    else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        exit(1);
    }
}

// 设备初始参数设置并打开
void set_CAN_para(VCI_INIT_CONFIG config)
{
    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
    //  if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config) != 1)
    {
        printf(">>Init can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
    {
        printf(">>Start can2 error\n");

        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
}

// 发送1帧数据，含有8个字节的数据指令
void send_CAN_data(VCI_CAN_OBJ *send)
{
    for (int t = 0; t < send_frame_length; t++)
    {
        // 利用CAN通道ind_send=1 发送数据
        if (VCI_Transmit(VCI_USBCAN2, 0, ind_send, &send[t], 1) != 1)
        {
            break;
        }
        usleep(1);
    }
}

// 接收数据
void *receive_CAN_func(void *)
{
    int reclen = 0;
    int read_sleep_ms = 0;
    for (int read_time = 0; read_time < 20; read_time++)
    {
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind_receive, rec, 3000, 100)) > 0) // 调用接收函数，如果有数据，进行数据处理显示。
        {
            for (int j = 0; j < reclen; j++)
            {

                for (int i = 0; i < rec[j].DataLen; i++)
                {
                }
            }
        }
        sleep(1);
    }
    pthread_exit(0);
}

PVCI_CAN_OBJ pspeed_cmd;
PVCI_CAN_OBJ pCAN_out_cmd;
PVCI_CAN_OBJ pmotor_unenable_cmd;

// 去使能，电机停转
void unenable_motor(PVCI_CAN_OBJ motor_unenable_cmd){
    // 这里传入的是首个信号的，如果直接send就只能控制1个轮子
    VCI_CAN_OBJ unenable_cmd[4]= {motor_unenable_cmd[0],motor_unenable_cmd[1],motor_unenable_cmd[2],motor_unenable_cmd[3]};  
    
    for (int t = 0; t < 20; t++){
        send_CAN_data(unenable_cmd);
        // send_CAN_data(motor_unenable_cmd);
        usleep(10);
    }
    printf("**************\n");
    printf("Motor unenabled \n");
    printf("**************\n");
    usleep(unenable_motor_usleep);
}


// 计算四个轮子平滑后的转速指令
bool get4WheelSpeedCmd(int speed_count_1, int speed_count_2, int speed_count_3, int speed_count_4,
                      PVCI_CAN_OBJ speed_cmd){
    speed_cmd[0].Data[4] = first_low8bits_from_int32(speed_count_1);
    speed_cmd[0].Data[5] = second_low8bits_from_int32(speed_count_1);
    speed_cmd[0].Data[6] = third_low8bits_from_int32(speed_count_1);
    speed_cmd[0].Data[7] = fourth_low8bits_from_int32(speed_count_1);

    speed_cmd[1].Data[4] = first_low8bits_from_int32(speed_count_2);
    speed_cmd[1].Data[5] = second_low8bits_from_int32(speed_count_2);
    speed_cmd[1].Data[6] = third_low8bits_from_int32(speed_count_2);
    speed_cmd[1].Data[7] = fourth_low8bits_from_int32(speed_count_2);

    speed_cmd[2].Data[4] = first_low8bits_from_int32(speed_count_3);
    speed_cmd[2].Data[5] = second_low8bits_from_int32(speed_count_3);
    speed_cmd[2].Data[6] = third_low8bits_from_int32(speed_count_3);
    speed_cmd[2].Data[7] = fourth_low8bits_from_int32(speed_count_3);

    speed_cmd[3].Data[4] = first_low8bits_from_int32(speed_count_4);
    speed_cmd[3].Data[5] = second_low8bits_from_int32(speed_count_4);
    speed_cmd[3].Data[6] = third_low8bits_from_int32(speed_count_4);
    speed_cmd[3].Data[7] = fourth_low8bits_from_int32(speed_count_4);

    return false;
}

// 抱死电机、停止CAN通信、停止遥控指令接受
void sigint_handler(int sig, PVCI_CAN_OBJ motor_unenable_cmd, PVCI_CAN_OBJ speed_cmd, PVCI_CAN_OBJ CAN_out_cmd)
{  
	// motor_unenable_cmd 作为指针被更改
    printf("程序异常终止, 电机终止。Received SIGINT signal %d\n", sig);
    // 调用你的回调函数来处理你想要执行的操作

    get4WheelSpeedCmd(0, 0, 0, 0, speed_cmd);  // output

    send_CAN_data(speed_cmd);
    usleep(sigint_handler_usleep);
    unenable_motor(motor_unenable_cmd);

    send_CAN_data(CAN_out_cmd);
    // exit_can(pCAN_out_cmd); // 全部退出CAN操作模式,CAN_out_cmd
    VCI_ResetCAN(VCI_USBCAN2, 0, 0); // 复位CAN1通道。
    usleep(sigint_handler_usleep);                  // 延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1); // 复位CAN2通道。
    usleep(sigint_handler_usleep);                  // 延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0); // 关闭设备。
    printf("CAN device closed ! \n");
    exit(1);
}

/// @brief 程序在Ctrl+C终止时自动运行下面这个程序
/// @return __sighandler_t
void funcPtr(int sig) {
    sigint_handler(sig, pmotor_unenable_cmd, pspeed_cmd, pCAN_out_cmd);
}

void get_vel(const geometry_msgs::Twist::ConstPtr &vel)
{    
    newvvv = true;
    float tv = vel->linear.x;
    float tw = vel->angular.x;
    vvvL = tv+rrr/2*tw;
    vvvR = tv-rrr/2*tw;//这里拿到的是以m为单位的速度
    vvvL = vvvL/0.754*60;//一圈是75.2cm,这里是rpm
    vvvR = vvvR/0.754*60;
}
//0.180746367
//0.319243225
//线速度0.25m/s
//角速度0.249994328
int main(int argc,char** argv)
{
    ros::init(argc,argv,"motor_node");
    ros::NodeHandle nh;
    ros::Subscriber Vel_listen = nh.subscribe<geometry_msgs::Twist>("vel_cmd",5,get_vel);

    printf(">>controlcan script start running !\r\n"); // 指示程序已运行

    find_CAN_device(); // 寻找设备
    open_CAN_device(); // 打开设备
    // 初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config_para;
    config_para.AccCode = 0;
    config_para.AccMask = 0xFFFFFFFF;
    config_para.Filter = 1;     // 接收所有帧
    config_para.Timing0 = 0x03; /*波特率125 Kbps  0x03  0x1C*/
    config_para.Timing1 = 0x1C;
    config_para.Mode = 0;      // 正常模式
    set_CAN_para(config_para); // 参数初始化并且打开通道
    printf("-----------------start CAN communicatioin ! ----------------\n");

    // 参数
    int motor_num = 0;
    int t = 0;
    int speed_rpm = 0;
    int speed_count_1 = 0;
    int speed_count_2 = 0;
    int speed_count_3 = 0;
    int speed_count_4 = 0;
    int speed_count_beforePaused[4] = {0,0,0,0};
    char KeyVal; // 键盘输入的键值

    // 四个电机先去使能
    VCI_CAN_OBJ motor_unenable_cmd[4];

    for (motor_num = 0; motor_num < 4; motor_num++)
    {
        motor_unenable_cmd[motor_num].SendType = 0;
        motor_unenable_cmd[motor_num].RemoteFlag = 0;
        motor_unenable_cmd[motor_num].ExternFlag = 0;
        motor_unenable_cmd[motor_num].DataLen = 8;
        motor_unenable_cmd[motor_num].Data[0] = 0x2B;
        motor_unenable_cmd[motor_num].Data[1] = 0x40;
        motor_unenable_cmd[motor_num].Data[2] = 0x60;
        motor_unenable_cmd[motor_num].Data[3] = 0x00;
        motor_unenable_cmd[motor_num].Data[4] = 0x00;
        motor_unenable_cmd[motor_num].Data[5] = 0x00;
        motor_unenable_cmd[motor_num].Data[6] = 0x00;
        motor_unenable_cmd[motor_num].Data[7] = 0x00;
    }
    motor_unenable_cmd[0].ID = 0x0000060F;//右前
    motor_unenable_cmd[1].ID = 0x0000060D;//左前
    motor_unenable_cmd[2].ID = 0x00000601;//左后
    motor_unenable_cmd[3].ID = 0x00000602;//右后
    for (t = 0; t < 20; t++)
    {
        send_CAN_data(motor_unenable_cmd);
        usleep(send_CAN_data_usleep);
    }
    printf("**************\n");
    printf("Motor unenabled \n");
    printf("**************\n");
    usleep(unenable_motor_usleep);

    pmotor_unenable_cmd = motor_unenable_cmd;
    signal(SIGINT, funcPtr);



    // 全部退出CAN操作模式
    VCI_CAN_OBJ CAN_out_cmd[1];
    CAN_out_cmd[0].SendType = 0;
    CAN_out_cmd[0].RemoteFlag = 0;
    CAN_out_cmd[0].ExternFlag = 0;
    CAN_out_cmd[0].ID = 0x00000000;
    CAN_out_cmd[0].DataLen = 2;
    CAN_out_cmd[0].Data[0] = 0x02;
    CAN_out_cmd[0].Data[1] = 0x00;
    for (t = 0; t < 20; t++)
    {
        send_CAN_data(CAN_out_cmd);
        usleep(send_CAN_data_usleep);
    }
    printf("**************\n");
    printf("CAN operation out\n");
    printf("**************\n");
    usleep(CAN_operation_out_cmd_usleep);
    pCAN_out_cmd = CAN_out_cmd;

    // 全部进入CAN操作模式
    VCI_CAN_OBJ CAN_operation_cmd[1];
    CAN_operation_cmd[0].SendType = 0;
    CAN_operation_cmd[0].RemoteFlag = 0;
    CAN_operation_cmd[0].ExternFlag = 0;
    CAN_operation_cmd[0].ID = 0x00000000;
    CAN_operation_cmd[0].DataLen = 2;
    CAN_operation_cmd[0].Data[0] = 0x01;
    CAN_operation_cmd[0].Data[1] = 0x00;
    for (t = 0; t < 20; t++)
    {
        send_CAN_data(CAN_operation_cmd);
        usleep(send_CAN_data_usleep);
    }
    printf("**************\n");
    printf("CAN operation mode\n");
    printf("**************\n");
    usleep(CAN_operation_out_cmd_usleep);

    // 四个电机进入速度模式
    VCI_CAN_OBJ speed_mode_cmd[4];
    for (motor_num = 0; motor_num < 4; motor_num++)
    {
        speed_mode_cmd[motor_num].SendType = 0;
        speed_mode_cmd[motor_num].RemoteFlag = 0;
        speed_mode_cmd[motor_num].ExternFlag = 0;
        speed_mode_cmd[motor_num].DataLen = 8;
        speed_mode_cmd[motor_num].Data[0] = 0x2F;
        speed_mode_cmd[motor_num].Data[1] = 0x60;
        speed_mode_cmd[motor_num].Data[2] = 0x60;
        speed_mode_cmd[motor_num].Data[3] = 0x00;
        speed_mode_cmd[motor_num].Data[4] = 0x03;
        speed_mode_cmd[motor_num].Data[5] = 0x00;
        speed_mode_cmd[motor_num].Data[6] = 0x00;
        speed_mode_cmd[motor_num].Data[7] = 0x00;
    }
    speed_mode_cmd[0].ID = 0x0000060F;
    speed_mode_cmd[1].ID = 0x0000060D;
    speed_mode_cmd[2].ID = 0x00000601;
    speed_mode_cmd[3].ID = 0x00000602;
    for (t = 0; t < 20; t++)
    {
        send_CAN_data(speed_mode_cmd);
        usleep(send_CAN_data_usleep);
    }

    printf("**************\n");
    printf("Speed mode enabled \n");
    printf("**************\n");

    // 四个电机使能
    VCI_CAN_OBJ motor_enable_cmd[4];
    for (motor_num = 0; motor_num < 4; motor_num++)
    {
        motor_enable_cmd[motor_num].SendType = 0;
        motor_enable_cmd[motor_num].RemoteFlag = 0;
        motor_enable_cmd[motor_num].ExternFlag = 0;
        motor_enable_cmd[motor_num].DataLen = 8;
        motor_enable_cmd[motor_num].Data[0] = 0x2B;
        motor_enable_cmd[motor_num].Data[1] = 0x40;
        motor_enable_cmd[motor_num].Data[2] = 0x60;
        motor_enable_cmd[motor_num].Data[3] = 0x00;
        motor_enable_cmd[motor_num].Data[4] = 0x0F;
        motor_enable_cmd[motor_num].Data[5] = 0x00;
        motor_enable_cmd[motor_num].Data[6] = 0x00;
        motor_enable_cmd[motor_num].Data[7] = 0x00;
    }
    motor_enable_cmd[0].ID = 0x0000060F;
    motor_enable_cmd[1].ID = 0x0000060D;
    motor_enable_cmd[2].ID = 0x00000601;
    motor_enable_cmd[3].ID = 0x00000602;
    for (t = 0; t < 20; t++)
    {
        send_CAN_data(motor_enable_cmd);
        usleep(send_CAN_data_usleep);
    }
    printf("**************\n");
    printf("Motor enabled \n");
    printf("**************\n");
    usleep(unenable_motor_usleep);

    VCI_CAN_OBJ speed_cmd[4];
    for (motor_num = 0; motor_num < 4; motor_num++)
    {
        speed_cmd[motor_num].SendType = 0;
        speed_cmd[motor_num].RemoteFlag = 0;
        speed_cmd[motor_num].ExternFlag = 0;
        speed_cmd[motor_num].DataLen = 8;
        speed_cmd[motor_num].Data[0] = 0x23;
        speed_cmd[motor_num].Data[1] = 0xff;
        speed_cmd[motor_num].Data[2] = 0x60;
        speed_cmd[motor_num].Data[3] = 0x00;

        speed_cmd[motor_num].Data[4] = 0x00;
        speed_cmd[motor_num].Data[5] = 0x00;
        speed_cmd[motor_num].Data[6] = 0x00;
        speed_cmd[motor_num].Data[7] = 0x00;
    }
    speed_cmd[0].ID = 0x0000060F;
    speed_cmd[1].ID = 0x0000060D;
    speed_cmd[2].ID = 0x00000601;
    speed_cmd[3].ID = 0x00000602;
    speed_cmd[0].Data[4] = first_low8bits_from_int32(speed_count_1);
    speed_cmd[0].Data[5] = second_low8bits_from_int32(speed_count_1);
    speed_cmd[0].Data[6] = third_low8bits_from_int32(speed_count_1);
    speed_cmd[0].Data[7] = fourth_low8bits_from_int32(speed_count_1);

    speed_cmd[1].Data[4] = first_low8bits_from_int32(speed_count_2);
    speed_cmd[1].Data[5] = second_low8bits_from_int32(speed_count_2);
    speed_cmd[1].Data[6] = third_low8bits_from_int32(speed_count_2);
    speed_cmd[1].Data[7] = fourth_low8bits_from_int32(speed_count_2);

    speed_cmd[2].Data[4] = first_low8bits_from_int32(speed_count_3);
    speed_cmd[2].Data[5] = second_low8bits_from_int32(speed_count_3);
    speed_cmd[2].Data[6] = third_low8bits_from_int32(speed_count_3);
    speed_cmd[2].Data[7] = fourth_low8bits_from_int32(speed_count_3);
    
    speed_cmd[3].Data[4] = first_low8bits_from_int32(speed_count_4);
    speed_cmd[3].Data[5] = second_low8bits_from_int32(speed_count_4);
    speed_cmd[3].Data[6] = third_low8bits_from_int32(speed_count_4);
    speed_cmd[3].Data[7] = fourth_low8bits_from_int32(speed_count_4);
    send_CAN_data(speed_cmd);

    pspeed_cmd = speed_cmd;
    
    cout << "USE KEYBOARD TO CONTROL ! >>>" << endl;
    bool paused = false,needToEnable = false, needToUnenable = true;
    clock_t startTime, currentTime;
    double elapsedTime;
    bool closeToCommand = true;
    ros::Rate rt(10);
    while (ros::ok()) // 循环检测键盘输入键值
    {
            ros::spinOnce();       
            if(newvvv)
            {
            speed_count_1 = -vvvL*2730.6666;
            speed_count_2 = vvvR*2730.6666;
            speed_count_3 = vvvR*2730.6666;
            speed_count_4 = -vvvL*2730.6666;
            printf("**************\n");            
            cout << "now vvvL is "<<vvvL<< "now vvvR is "<<vvvR<< endl;
            closeToCommand = get4WheelSpeedCmd(speed_count_1,speed_count_2,speed_count_3, speed_count_4, speed_cmd);  // output
            send_CAN_data(speed_cmd);
            newvvv = false;
            }
            rt.sleep();       
    }       
}
/*

        if (KeyVal == 'r')//重启
        {
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(motor_unenable_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            printf("Motor unenabled \n");
            printf("**************\n");
            usleep(unenable_motor_usleep);
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(CAN_out_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            printf("CAN operation exit\n");
            printf("**************\n");
            usleep(for_CAN_operation_cmd_usleep);
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(CAN_operation_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            printf("CAN operation mode\n");
            printf("**************\n");
            usleep(for_CAN_operation_cmd_usleep);
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(motor_enable_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            cout << ">>>re-enable ! " << endl;
            printf("**************\n");
            paused = false;
            usleep(unenable_motor_usleep);
        }
        if (KeyVal == 'q')//终止
        {
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(motor_unenable_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            printf("Motor unenabled \n");
            printf("**************\n");
            cout << ">>>stop all motors and exit ! " << endl;
            printf("**************\n");
            sleep(1);
            break;
        }
        if (KeyVal == '0')//停止
        {
            speed_rpm = 0;
            speed_count_1 = abs(speed_rpm) * 10 * 4096 / 60;
            speed_count_2 = abs(speed_rpm) * 10 * 4096 / 60;
            speed_count_3 = abs(speed_rpm) * 10 * 4096 / 60;
            speed_count_4 = abs(speed_rpm) * 10 * 4096 / 60;
            printf("**************\n");
            cout << ">>>stop all motors ! " << endl;
            printf("**************\n");
        }
        if ((KeyVal == '=')&&!paused)//暂停、轮子断电
        {
            speed_count_beforePaused[0] = speed_count_1;
            speed_count_beforePaused[1] = speed_count_2;
            speed_count_beforePaused[2] = speed_count_3;
            speed_count_beforePaused[3] = speed_count_4;
            speed_count_1 = 0;
            speed_count_2 = 0;
            speed_count_3 = 0;
            speed_count_4 = 0;
            
            printf("Motor paused \n");
            printf("**************\n");
            usleep(pause_usleep);
            paused = true;needToEnable=true;
            // t0 = tf;
        }
        if(paused&&needToEnable){
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(motor_unenable_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            printf("Motor unenabled \n");
            needToEnable = false;
        }
        if ((KeyVal == '/')&&paused)//继续、轮子上电
        {
            speed_count_1 = speed_count_beforePaused[0];
            speed_count_2 = speed_count_beforePaused[1];
            speed_count_3 = speed_count_beforePaused[2];
            speed_count_4 = speed_count_beforePaused[3];
            
            printf("Motor started \n");
            printf("**************\n");
            usleep(pause_usleep);
            paused = false;needToUnenable=true;
            // t0 = tf;
        }
        if(!paused&&needToUnenable){
            for (t = 0; t < 20; t++)
            {
                send_CAN_data(motor_enable_cmd);
                usleep(for_send_CAN_data_usleep);
            }
            printf("**************\n");
            printf("Motor enabled \n");
            needToUnenable = false;
        }
        if (KeyVal == '5')//前进
        {   // 线圈数量4096 4倍频，因此以转对应16384count,而电机分辨率0.1count/s所以10rpm的速度，则数值为 10/60(多少转每秒)*16384(多少count/s)*10(多少分辨率/s)=10(rpm)*   
            speed_rpm = 10;
            speed_count_1 = -abs(speed_rpm)*2730.6666;
            speed_count_2 =  abs(speed_rpm)*2730.6666;
            speed_count_3 =  abs(speed_rpm)*2730.6666;
            speed_count_4 = -abs(speed_rpm)*2730.6666;
            printf("**************\n");
            cout << "now rpm is " << speed_rpm <<"time is "<< endl;
        }
        if (KeyVal == '2')//后退
        {   
            speed_rpm = 30;
            speed_count_1 = abs(speed_rpm)*2730.6666;
            speed_count_2 = -abs(speed_rpm)*2730.6666;
            speed_count_3 = -abs(speed_rpm)*2730.6666;                                                                                                                                                                                                                                                                                  speed_count_3 = -abs(times*speed_rpm) * 10 * 4096 / 60;
            speed_count_4 = abs(speed_rpm)*2730.6666;
            // send_CAN_data(speed_cmd);
            printf("**************\n");
            cout << "now rpm is " << speed_rpm <<"time is "<< endl;
        }
        if (KeyVal == '3')//右转
        {
            speed_count_1 = abs(speed_rpm) * 5 * 4096 / 60;
            speed_count_2 = abs(speed_rpm) * 5 * 4096 / 60;
            speed_count_3 = abs(speed_rpm) * 5 * 4096 / 60;
            speed_count_4 = abs(speed_rpm) * 5 * 4096 / 60;
        }
        if (KeyVal == '1')//左转
        {
            speed_count_1 = -abs(speed_rpm) * 5 * 4096 / 60;
            speed_count_2 = -abs(speed_rpm) * 5 * 4096 / 60;
            speed_count_3 = -abs(speed_rpm) * 5 * 4096 / 60;
            speed_count_4 = -abs(speed_rpm) * 5 * 4096 / 60;
        }
*/



    // for (t = 0; t < 20; t++)
    // {
    //     send_CAN_data(motor_unenable_cmd);
    //     usleep(send_CAN_data_usleep);
    // }
    // printf("**************\n");
    // printf("Motor unenabled \n");
    // printf("**************\n");
    // usleep(unenable_motor_usleep);
    // printf("------------------finish CAN communication ! -----------------\n");
    // sleep(1);
    // VCI_ResetCAN(VCI_USBCAN2, 0, 0); // 复位CAN1通道。
    // usleep(end_usleep);                  // 延时100ms。
    // VCI_ResetCAN(VCI_USBCAN2, 0, 1); // 复位CAN2通道。
    // usleep(end_usleep);                  // 延时100ms。
    // VCI_CloseDevice(VCI_USBCAN2, 0); // 关闭设备。
    // printf("CAN device closed ! \n");
    // return (0);
