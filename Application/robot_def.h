/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人定义,包含机器人的各种参数
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // ! 单板控制整车，beta选项，建议别选上
#define CHASSIS_BOARD // 底盘板，注意底盘板还控制了云台的YAW轴
// #define GIMBAL_BOARD // 云台板

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 底盘参数
#define CHASSIS_OMNI_WHEEL // 是否为全向轮底盘
// #define CHASSIS_MCNAMEE_WHEEL // 是否为麦克纳姆轮底盘

#define VISION_USE_VCP // 是否使用虚拟串口
// #define VISION_USE_UART // 是否使用硬件串口

// #define VIDEO_LINKK // 是否有图传链路
#define REMOTE_LINK // 是否有常规链路

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// 检查是否出现底盘类型定义冲突,只允许一个底盘类型定义存在,否则编译会自动报错
#if (defined(CHASSIS_OMNI_WHEEL) && defined(CHASSIS_MCNAMEE_WHEEL))
#error Conflict chassis definition! You can only define one chassis type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum {
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum {
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,pc在云台,遥控器和裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// 双板时，下板cmd发布控制云台控制数据，由gimbal订阅
typedef struct
{
    float yaw;
    float up_yaw;
    float up_speed;

    uint8_t is_init;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Yaw_Cmd_s;

typedef struct
{
    float pitch;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Pitch_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

// 上C -> 下C
typedef struct
{
    Gimbal_Ctrl_Yaw_Cmd_s gimbal_cmd; // 视觉反馈的云台控制命令
    float yaw;                        // 云台yaw角度
    float speed;                      // 云台yaw轴速度
} Up_To_Down_Data_s;

// 下C -> 上C
typedef struct
{
    Gimbal_Ctrl_Pitch_Cmd_s gimbal_cmd; // 底盘反馈的云台控制命令，用于控制Pitch轴
    // 发射任务命令
} Down_To_Up_Data_s;

#pragma pack() // 取消压缩
#endif