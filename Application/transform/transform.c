#include "transform.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"

static DJIMotor_Instance *trans;

// static Publisher_t *trans_pub;  // 用于发布传送带的数据
// static Subscriber_t

void TransformInit(void)
{
    // 初始化变换模块
    Motor_Init_Config_s trans_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 5, // 10
                .Ki            = 0, // 1
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
            .current_PID = {
                .Kp            = 0.7, // 0.7
                .Ki            = 0,   // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP, // 初始化成SPEED_LOOP,
            .close_loop_type    = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向
        },
        .motor_type = M2006};

    trans = DJIMotorInit(&trans_motor_config);
};

void TransformTask(void)
{
    // 传送带任务
    DJIMotorSetRef(trans, 10000);
}