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
#include "miniPC_process.h"
#include "referee_protocol.h"
#include "scara_kinematics.h"
#include "arm_math.h"
#include "UARTComm.h"

static float rc_mode_xy[2]             = {0, 0}; // x,y坐标
static float rc_mode_xy_after_check[2] = {0, 0};
static float yaw_offset                = 0;

static Publisher_t *arm_cmd_pub;         // 底盘控制消息发布者
static Subscriber_t *arm_feed_sub;       // 底盘反馈信息订阅者
static Vision_Recv_s *vision_ctrl;       // 视觉控制信息
static UARTComm_Instance *cmd_uart_comm; // 双板通信

static Chassis_Ctrl_Cmd_s chassis_cmd_send;       // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data;  // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Arm_Ctrl_Cmd_s arm_cmd_send;               // 发送给机械臂的信息
__unused static Arm_Upload_Data_s arm_fetch_data; // 从机械臂接受的反馈信息
static void RemoteControlSet(void);               // 遥控器控制量设置
static void VideoControlSet(void);                // 图传链路控制量设置
static RC_ctrl_t *rc_data;                        // 遥控器数据指针,初始化时返回
static Video_ctrl_t *video_data;                  // 视觉数据指针,初始化时返回

float scara_height;         //@bi
uint8_t crazy_chassis_flag; //@bi
pid_type_def PID_lift;      //鉴于上一个写代码风格的过于抽象，只好自己写一个

static void VisionContorl(void);
/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 * @attention 工程机器人使用两块板子，它们之间的数据使用UART 1进行通信
 *
 */
void RobotCMDInit(void)
{

    UARTComm_Init_Config_s ucomm_config = {
        .uart_handle    = &huart1,
        .recv_data_len  = sizeof(Chassis_Upload_Data_s),
        .send_data_len  = sizeof(Chassis_Ctrl_Cmd_s),
        .daemon_counter = 10,
    };
    cmd_uart_comm = UARTCommInit(&ucomm_config);
    video_data    = VideoTransmitterControlInit(&huart6); // 初始化图传链路
    rc_data       = RemoteControlInit(&huart3);           // 初始化遥控器,C板上使用USART1
    vision_ctrl   = VisionInit(&huart3);                  // 初始化视觉控制

    arm_cmd_pub  = PubRegister("arm_cmd", sizeof(Arm_Ctrl_Cmd_s));
    arm_feed_sub = SubRegister("arm_feed", sizeof(Arm_Upload_Data_s));

    // 此处初始化与视觉的通信
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    // 获取机械臂反馈信息
    SubGetMessage(arm_feed_sub, &arm_fetch_data);
    chassis_fetch_data = *(Chassis_Upload_Data_s *)UARTCommGet(cmd_uart_comm);

    if (!rc_data[TEMP].rc.switch_right ||
        switch_is_down(rc_data[TEMP].rc.switch_right)) // 当收不到遥控器信号时，使用图传链路
    {
        VideoControlSet();
    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right) || switch_is_up(rc_data[TEMP].rc.switch_right)) // 当收到遥控器信号时,且右拨杆为中，使用遥控器
    {
        RemoteControlSet();
    }

    // 发送控制信息
    chassis_cmd_send.arm_mode    = arm_cmd_send.arm_mode;
    chassis_cmd_send.sucker_mode = arm_cmd_send.sucker_mode;
    chassis_cmd_send.arm_status  = arm_cmd_send.arm_status;
    chassis_cmd_send.max_arm     = arm_fetch_data.maximal_arm;
    chassis_cmd_send.min_arm     = arm_fetch_data.minimal_arm;

    // 发送给机械臂
    PubPushMessage(arm_cmd_pub, &arm_cmd_send);
    UARTCommSend(cmd_uart_comm, (void *)&chassis_cmd_send);
}

static void RemoteControlSet(void)
{
    PID_init(&PID_lift, PID_POSITION, 500, 30, 0, 50000, 30000);
    chassis_cmd_send.chassis_mode = CHASSIS_SLOW;
    arm_cmd_send.arm_mode         = ARM_HUM_CONTORL;
    float res_scara_angle[2]; //第一个为大臂，第二个为小臂
    uint8_t switch_left_down_flag;
    uint8_t switch_left_up_flag;
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
        switch_left_down_flag = 1;
        switch_left_up_flag   = 0;
    } else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
        switch_left_down_flag = 0;
        switch_left_up_flag   = 1;
    } else {
        switch_left_down_flag = 0;
        switch_left_up_flag   = 0;
    }
    //图传角度
    if (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2 == 1) {
        arm_cmd_send.video_angle = PITCH_120;
    } else {
        arm_cmd_send.video_angle = PITCH_90;
    }
    //取银矿模式
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].x) {
        rc_mode_xy[0]          = 250;
        arm_cmd_send.pitch_arm = -PI / 2;
        yaw_offset             = 0;
    }
    //取金矿模式
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].c) {
        rc_mode_xy[1]          = 0;
        arm_cmd_send.pitch_arm = 0;
        yaw_offset             = 0;
    }
    //前两轴解算部分
    if ((!(rc_data[TEMP].key[KEY_PRESS].ctrl)) && (!(rc_data[TEMP].mouse.press_l && rc_data[TEMP].mouse.press_r))) {
        rc_mode_xy[1] -= rc_data[TEMP].mouse.x / 50;
        rc_mode_xy[0] -= rc_data[TEMP].mouse.y / 30;
    }
    if (rc_data[TEMP].rc.rocker_r1 > 100) {
        rc_mode_xy[0] += (rc_data[TEMP].rc.rocker_r1) / 240;
    }
    if (rc_data[TEMP].rc.rocker_r1 < -100) {
        rc_mode_xy[0] += (rc_data[TEMP].rc.rocker_r1) / 240;
    }
    if (rc_data[TEMP].rc.rocker_r_ > 100) {
        rc_mode_xy[1] -= (rc_data[TEMP].rc.rocker_r_) / 240;
    }
    if (rc_data[TEMP].rc.rocker_r_ < -100) {
        rc_mode_xy[1] -= (rc_data[TEMP].rc.rocker_r_) / 240;
    }
    // x限位
    if (rc_mode_xy[0] > ARMLENGHT1 + ARMLENGHT2) {
        rc_mode_xy[0] = ARMLENGHT1 + ARMLENGHT2;
    }
    if (rc_mode_xy[0] < 0) {
        rc_mode_xy[0] = 0;
    }
    // y限位
    if (rc_mode_xy[1] > 400) {
        rc_mode_xy[1] = 400;
    }
    if (rc_mode_xy[1] < -228) {
        rc_mode_xy[1] = -228;
    }
    check_boundary_scara_lefthand(rc_mode_xy[0], rc_mode_xy[1], rc_mode_xy_after_check);
    scara_inverse_kinematics(rc_mode_xy_after_check[0], rc_mode_xy_after_check[1], ARMLENGHT1, ARMLENGHT2, 2, res_scara_angle);
    arm_cmd_send.maximal_arm = res_scara_angle[0];
    arm_cmd_send.minimal_arm = res_scara_angle[1];
    //末端三轴
    // yaw pitch
    if ((!(rc_data[TEMP].mouse.press_r && rc_data[TEMP].mouse.press_l && rc_data[TEMP].key[KEY_PRESS].shift)) && (rc_data[TEMP].key[KEY_PRESS].ctrl)) {
        if (rc_data[TEMP].mouse.x > 10) {
            yaw_offset -= 0.01;
        }
        if (rc_data[TEMP].mouse.x < -10) {
            yaw_offset += 0.01;
        }
        if (rc_data[TEMP].mouse.y > 10) {
            arm_cmd_send.pitch_arm -= 0.01;
        }
        if (rc_data[TEMP].mouse.y < -10) {
            arm_cmd_send.pitch_arm += 0.01;
        }
    }
    if (yaw_offset > PI) {
        yaw_offset = PI;
    }
    if (yaw_offset < -PI) {
        yaw_offset = -PI;
    }
    if (arm_cmd_send.pitch_arm < -PI / 2) {
        arm_cmd_send.pitch_arm = -PI / 2;
    }
    if (arm_cmd_send.pitch_arm > PI / 2) {
        arm_cmd_send.pitch_arm = PI / 2;
    }
    arm_cmd_send.finesse = yaw_offset - res_scara_angle[0] - res_scara_angle[1];
    if ((rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2 == 1) || switch_is_up(rc_data[TEMP].rc.switch_right)) {
        arm_cmd_send.sucker_mode = SUCKER_ON;
    } else {
        arm_cmd_send.sucker_mode = SUCKER_OFF;
    }
    //一键放矿
    if (rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_B] % 2 == 1) {
        chassis_cmd_send.trans_mode = TRANS_REVERSE;
        if (rc_mode_xy[1] < 400) {
            rc_mode_xy[0] = 250;
            rc_mode_xy[1] += 4;
            arm_cmd_send.pitch_arm = -PI / 2;
            yaw_offset             = 0;
        } else if (rc_mode_xy[1] == 400) {
            arm_cmd_send.maximal_arm = 1.42956f;
            arm_cmd_send.minimal_arm = 2.01921f;
            arm_cmd_send.finesse     = 0.f + yaw_offset;
            arm_cmd_send.pitch_arm   = -PI / 2;
        }
        arm_cmd_send.video_angle = PITCH_120;
    }
    if (rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_G] % 2 == 1) {
        chassis_cmd_send.trans_mode = TRANS_DIRECT;
        if (rc_mode_xy[1] < 400) {
            rc_mode_xy[0] = 250;
            rc_mode_xy[1] += 4;
            arm_cmd_send.pitch_arm = -PI / 2;
            yaw_offset             = 0;
        } else if (rc_mode_xy[1] == 400) {
            arm_cmd_send.maximal_arm = 1.42956f;
            arm_cmd_send.minimal_arm = 2.01921f;
            arm_cmd_send.finesse     = 0.f + yaw_offset;
            arm_cmd_send.pitch_arm   = -PI / 2;
        }
        arm_cmd_send.video_angle = PITCH_120;
    }
    if (rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_G] % 2 == 0 && rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_B] % 2 == 0) {
        chassis_cmd_send.trans_mode = TRANS_STOP;
    }
    //抬升
    if (rc_data[TEMP].mouse.press_l && (!rc_data[TEMP].mouse.press_r)) {
        arm_cmd_send.lift += 500;
    } else if (rc_data[TEMP].mouse.press_r && (!rc_data[TEMP].mouse.press_l)) {
        arm_cmd_send.lift -= 500;
    } else {
        arm_cmd_send.lift = 0;
    }
    if (switch_left_down_flag) {
        arm_cmd_send.lift = 10000;
    }
    if (switch_left_up_flag) {
        arm_cmd_send.lift = -10000;
    }
    if (arm_cmd_send.lift > 50000) {
        arm_cmd_send.lift = 50000;
    }
    if (arm_cmd_send.lift < -50000) {
        arm_cmd_send.lift = -50000;
    }
    if (rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_Z] % 3 == 1) {
        arm_cmd_send.lift_mode = LIFT_SPEED_MODE;
        arm_cmd_send.lift      = -PID_calc(&PID_lift, scara_height, 120);
    }
    if (rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_Z] % 3 == 2) {
        arm_cmd_send.lift_mode = LIFT_SPEED_MODE;
        arm_cmd_send.lift      = -PID_calc(&PID_lift, scara_height, 165);
    }
    if (arm_cmd_send.lift == 0) {
        arm_cmd_send.lift_mode = LIFT_KEEP;
    } else {
        arm_cmd_send.lift_mode = LIFT_SPEED_MODE;
    }
    // roll
    if (rc_data[TEMP].key[KEY_PRESS].q || rc_data[TEMP].key[KEY_PRESS].e) {
        arm_cmd_send.roll_mode = ROLL_ANGLE_MODE;
    } else {
        arm_cmd_send.roll_mode = ROLL_KEEP;
    }
    arm_cmd_send.roll = ((30.f * rc_data[TEMP].key[KEY_PRESS].e) - (30.f * rc_data[TEMP].key[KEY_PRESS].q)) + arm_fetch_data.roll;
    int speed_scale   = 4;
    if (rc_data[TEMP].key[KEY_PRESS].f) {
        speed_scale = 100;
    }
    //平移缓启动
    if ((rc_data[TEMP].key[KEY_PRESS].d && (!rc_data[TEMP].key[KEY_PRESS].a)) || (rc_data[TEMP].rc.rocker_l_ > 200)) {
        chassis_cmd_send.vx += 10 * speed_scale;
    }
    if ((rc_data[TEMP].key[KEY_PRESS].a && (!rc_data[TEMP].key[KEY_PRESS].d)) || (rc_data[TEMP].rc.rocker_l_ < -200)) {
        chassis_cmd_send.vx -= 10 * speed_scale;
    }
    if ((!rc_data[TEMP].key[KEY_PRESS].a) && (!rc_data[TEMP].key[KEY_PRESS].d) && (fabs(rc_data[TEMP].rc.rocker_l_) < 200)) {
        chassis_cmd_send.vx = 0;
    }
    if (chassis_cmd_send.vx > 4000 * speed_scale) {
        chassis_cmd_send.vx = 4000 * speed_scale;
    }
    if (chassis_cmd_send.vx < -4000 * speed_scale) {
        chassis_cmd_send.vx = -4000 * speed_scale;
    }
    //前进缓启动
    if ((rc_data[TEMP].key[KEY_PRESS].w && (!rc_data[TEMP].key[KEY_PRESS].s)) || (rc_data[TEMP].rc.rocker_l1 > 200)) {
        chassis_cmd_send.vy += 300 * speed_scale;
    }
    if ((rc_data[TEMP].key[KEY_PRESS].s && (!rc_data[TEMP].key[KEY_PRESS].w)) || (rc_data[TEMP].rc.rocker_l1 < -200)) {
        chassis_cmd_send.vy -= 300 * speed_scale;
    }
    if ((!rc_data[TEMP].key[KEY_PRESS].s) && (!rc_data[TEMP].key[KEY_PRESS].w) && (fabs(rc_data[TEMP].rc.rocker_l1) < 200)) {
        chassis_cmd_send.vy = 0;
    }
    if (chassis_cmd_send.vy > 15000 * speed_scale) {
        chassis_cmd_send.vy = 15000 * speed_scale;
    }
    if (chassis_cmd_send.vy < -15000 * speed_scale) {
        chassis_cmd_send.vy = -15000 * speed_scale;
    }
    if (rc_data[TEMP].mouse.press_r && rc_data[TEMP].mouse.press_l) {
        chassis_cmd_send.wz = (float)rc_data[TEMP].mouse.x * 660;
    } else if (fabs(rc_data[TEMP].rc.dial) < 200) {
        chassis_cmd_send.wz = 0;
    } else {
        chassis_cmd_send.wz = -10.0f * (float)rc_data[TEMP].rc.dial;
    }
    if (rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].x) {
        VisionContorl();
        //更新位置
        scara_forward_kinematics(arm_fetch_data.maximal_arm, arm_fetch_data.minimal_arm, ARMLENGHT1, ARMLENGHT2, rc_mode_xy);
        yaw_offset = arm_fetch_data.maximal_arm + arm_fetch_data.minimal_arm + arm_fetch_data.finesse;
    }
    arm_cmd_send.arm_mode_last = arm_cmd_send.arm_mode;
}

/**
 * @brief 视觉控制
 *
 */
static void VisionContorl(void)
{
    arm_cmd_send.arm_mode = ARM_VISION_CONTROL;
    if (arm_cmd_send.arm_mode != arm_cmd_send.arm_mode_last) {
        VisionSend(1);
    }
    if (video_data[TEMP].key[KEY_PRESS].v) {
        VisionSend(1);
    } else {
        VisionSend(0);
    }
    if (vision_ctrl->is_tracking) {
        arm_cmd_send.maximal_arm = vision_ctrl->maximal_arm + MAXARM_ZERO;
        arm_cmd_send.minimal_arm = vision_ctrl->minimal_arm + MINARM_ZERO;
        arm_cmd_send.finesse     = vision_ctrl->finesse + FINE_ZERO;
        arm_cmd_send.pitch_arm   = vision_ctrl->pitch_arm + PITCH_ZERO;
    } else {
        arm_cmd_send.maximal_arm = arm_fetch_data.maximal_arm;
        arm_cmd_send.minimal_arm = arm_fetch_data.minimal_arm;
        arm_cmd_send.finesse     = arm_fetch_data.finesse;
        arm_cmd_send.pitch_arm   = arm_fetch_data.pitch_arm;
        arm_cmd_send.lift        = arm_fetch_data.height;
        arm_cmd_send.roll        = arm_fetch_data.roll;
        arm_cmd_send.lift_mode   = LIFT_KEEP;
        arm_cmd_send.roll_mode   = ROLL_KEEP;
        arm_cmd_send.arm_status  = ARM_NORMAL;
    }

    if (video_data[TEMP].key_data.left_button_down || video_data[TEMP].key_data.right_button_down) {
        arm_cmd_send.lift_mode = LIFT_ANGLE_MODE;
    }
    arm_cmd_send.lift = (3 * (video_data[TEMP].key_data.left_button_down) - 6 * (video_data[TEMP].key_data.right_button_down)) * 10 + arm_fetch_data.height;
    if (video_data[TEMP].key[KEY_PRESS].f || video_data[TEMP].key[KEY_PRESS].g) {
        arm_cmd_send.roll_mode = ROLL_ANGLE_MODE;
    } else {
        arm_cmd_send.roll_mode = ROLL_KEEP;
    }
    arm_cmd_send.roll          = arm_fetch_data.roll + 5 * (video_data[TEMP].key[KEY_PRESS].f - video_data[TEMP].key[KEY_PRESS].g);
    arm_cmd_send.arm_mode_last = arm_cmd_send.arm_mode;
}

/**
 * @brief 图传链路以及自定义控制器的模式和控制量设置
 *
 */

static void VideoControlSet(void)
{
    ;
}
