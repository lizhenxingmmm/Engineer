/**
 * @file robot_cmd.c
 * @author your name (you@domain.com)
 * @brief 机器人核心控制任务
 * @attention 因为底盘接的是遥控器，但是云台需要进行视觉传输，因此在预编译时不应该屏蔽掉RobotCMDInit，
 *             否则会出现内存访问错误，应在该文件中修改预编译指令。
 *             由于底盘板和云台板的任务都包含有云台电机的任务，因此应该在此处进行双板之间的通信。
 * @version 0.1
 * @date 2024-01-15
 *
 * @copyright Copyright (c) 2024
 *
 */
// application layer for robot command
#include "robot_cmd.h"
#include "robot_def.h"

// module layer
#include "remote.h"
#include "miniPC_process.h"
#include "VideoTransmitter.h"
#include "message_center.h"
#include "user_lib.h"
#ifdef CHASSIS_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
#endif
#ifdef ARM_BOARD
static Publisher_t *arm_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *arm_feed_sub; // 底盘反馈信息订阅者

static Arm_Ctrl_Cmd_s arm_cmd_send;      // 发送给机械臂的信息
static Arm_Upload_Data_s arm_fetch_data; // 从机械臂接受的反馈信息
#endif

static void RemoteControlSet(void); // 遥控器控制量设置
static void VideoControlSet(void);  // 图传链路控制量设置
static void EmergencyHandler(void);
static RC_ctrl_t *rc_data;       // 遥控器数据指针,初始化时返回
static Video_ctrl_t *video_data; // 视觉数据指针,初始化时返回

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 * @attention 工程机器人使用两块板子，它们之间的数据使用UART 1进行通信
 *
 */
void RobotCMDInit(void)
{
#ifdef ARM_BOARD
    video_data = VideoTransmitterControlInit(&huart6); // 初始化图传链路
    rc_data    = RemoteControlInit(&huart1);           // 初始化遥控器,C板上使用USART1

    arm_cmd_pub  = PubRegister("arm_cmd", sizeof(Arm_Ctrl_Cmd_s));
    arm_feed_sub = SubRegister("arm_feed", sizeof(Arm_Upload_Data_s));
#endif // ARM_BOARD

#ifdef CHASSIS_BOARD
    // 初始化遥控器,使用串口3
    rc_data          = RemoteControlInit(&huart3);           // 初始化遥控器,C板上使用USART3
    video_data       = VideoTransmitterControlInit(&huart1); // 初始化图传链路
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // CHASSIS_BOARD

    // 此处初始化与视觉的通信
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    // 获取各个模块的数据
#ifdef CHASSIS_BOARD
    // 获取底盘反馈信息
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
#endif
#ifdef ARM_BOARD
    // 获取机械臂反馈信息
    SubGetMessage(arm_feed_sub, &arm_fetch_data);
#endif
    if (!rc_data[TEMP].rc.switch_right ||
        switch_is_down(rc_data[TEMP].rc.switch_right)) // 当收不到遥控器信号时，使用图传链路
        VideoControlSet();
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 当收到遥控器信号时,且右拨杆为中，使用遥控器
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
        EmergencyHandler();

        // 发送控制信息
#ifdef ARM_BOARD
    // 发送给机械臂
    PubPushMessage(arm_cmd_pub, &arm_cmd_send);
#endif
#ifdef CHASSIS_BOARD
    // 发送给底盘
    PubPushMessage(chassis_cmd_pub, &chassis_cmd_send);
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet(void)
{
}

/**
 * @brief 图传链路以及自定义控制器的模式和控制量设置
 *
 */
static void VideoControlSet(void)
{
    // 直接测试，稍后会添加到函数中
#ifdef ARM_BOARD
    arm_cmd_send.maximal_arm = video_data[TEMP].cus.maximal_arm_target;
    arm_cmd_send.minimal_arm = video_data[TEMP].cus.minimal_arm_target;

    arm_cmd_send.finesse   = video_data[TEMP].cus.finesse_target;
    arm_cmd_send.pitch_arm = video_data[TEMP].cus.pitch_arm_target;

    VAL_LIMIT(arm_cmd_send.maximal_arm, -1.0f, 0.74f);
    VAL_LIMIT(arm_cmd_send.minimal_arm, -2.0f, 2.7f);
    VAL_LIMIT(arm_cmd_send.finesse, -1.6f, 1.9f);
    VAL_LIMIT(arm_cmd_send.pitch_arm, -0.8f, 1.0f);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_send.chassis_mode       = CHASSIS_SLOW;
    chassis_cmd_send.vx                 = (video_data[TEMP].key[KEY_PRESS].a - video_data[TEMP].key[KEY_PRESS].d) * 2000 * chassis_cmd_send.chassis_speed_buff; // 系数待测
    chassis_cmd_send.vy                 = (video_data[TEMP].key[KEY_PRESS].w - video_data[TEMP].key[KEY_PRESS].s) * 2000 * chassis_cmd_send.chassis_speed_buff; // 系数待测                                                                                                         // test
    chassis_cmd_send.wz                 = video_data[TEMP].key_data.mouse_x * 10 + (-video_data[TEMP].key[KEY_PRESS].q + video_data[TEMP].key[KEY_PRESS].e) * 500 * chassis_cmd_send.chassis_speed_buff;
    chassis_cmd_send.chassis_speed_buff = 1; // test
#endif
}

/**
 * @brief  紧急停止,包括遥控器右侧上侧拨杆打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void)
{
// 底盘急停
#ifdef CHASSIS_BOARD
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
#endif
    // 机械臂急停
#ifdef ARM_BOARD
    arm_cmd_send.arm_mode = ARM_ZERO_FORCE;
#endif
}
