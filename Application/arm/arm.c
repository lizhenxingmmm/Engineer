#include "arm.h"
#include "robot_def.h"

#include "dmmotor.h"
#include "message_center.h"
#include "user_lib.h"
// 大臂 -0.19 小臂 -0.79就寄啦？

// 图传链路 大臂零位maximal_arm -0.96 小臂零位minimal_arm 0.43 手腕零位 finesse 0.03 pitch_arm 零位 0.66
static Publisher_t *arm_pub;  // 用于发布底盘的数据
static Subscriber_t *arm_sub; // 用于订阅底盘的控制命令

static Arm_Ctrl_Cmd_s arm_cmd_recv;         // 发送给底盘的控制命令
static Arm_Upload_Data_s arm_feedback_data; // 从底盘接收的数据

static DM_MotorInstance *maximal_arm, *minimal_arm, *finesse, *pitch_arm;
static int8_t is_init;

static void ArmDMInit(void) // 非常抽象的函数，达妙电机不给值会回到原位，当然可以重新置零，但是工程的机械臂要限位？
{
    DMMotorControlInit();
    DMMotorSetRef(maximal_arm, maximal_arm->measure.position);
    DMMotorSetSpeedRef(maximal_arm, 0.3);

    DMMotorSetRef(minimal_arm, minimal_arm->measure.position);
    DMMotorSetSpeedRef(minimal_arm, 0.3);

    DMMotorSetRef(finesse, finesse->measure.position);
    DMMotorSetSpeedRef(finesse, 0.3);

    DMMotorSetRef(pitch_arm, pitch_arm->measure.position);
    DMMotorSetSpeedRef(pitch_arm, 0.3);

    arm_sub = SubRegister("arm_cmd", sizeof(Arm_Ctrl_Cmd_s));
    arm_pub = PubRegister("arm_feed", sizeof(Arm_Upload_Data_s));
}

void ArmInit(void)
{
    // 初始化机械臂
    Motor_Init_Config_s motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .rx_id      = 0x03, // Master ID 从3开始，因为发送id不能与接收id相同，
            .tx_id      = 1,    // MIT模式下为id，速度位置模式为0x100 + id
        },
        // 速度位置模式下不需要PID,喵老板真棒^^
        .control_type = MOTOR_CONTROL_POSITION_AND_SPEED,
        .motor_type   = DM4310,
    };
    maximal_arm = DMMotorInit(&motor_config);

    motor_config.can_init_config.rx_id = 0x04;
    motor_config.can_init_config.tx_id = 2;
    minimal_arm                        = DMMotorInit(&motor_config);

    motor_config.can_init_config.can_handle = &hcan2;
    motor_config.can_init_config.rx_id      = 0x03;
    motor_config.can_init_config.tx_id      = 1;
    finesse                                 = DMMotorInit(&motor_config);

    motor_config.can_init_config.rx_id = 0x04;
    motor_config.can_init_config.tx_id = 2;
    pitch_arm                          = DMMotorInit(&motor_config);
}

void ARMTask(void)
{
    if (!is_init) {
        DMMotorControlInit();
        ArmDMInit();
        is_init = 1;
    }
    SubGetMessage(arm_sub, &arm_cmd_recv);
    // 机械臂控制任务
    VAL_LIMIT(arm_cmd_recv.maximal_arm, -1.0f, 0.74f);
    DMMotorSetRef(maximal_arm, arm_cmd_recv.maximal_arm); // MIN -1.0,MAX 0.75

    VAL_LIMIT(arm_cmd_recv.minimal_arm, -2.0f, 2.7f);
    DMMotorSetRef(minimal_arm, arm_cmd_recv.minimal_arm); // MIN -2.0,MAX 2.7

    VAL_LIMIT(arm_cmd_recv.finesse, -1.6f, 1.9f);
    DMMotorSetRef(finesse, arm_cmd_recv.finesse); // MIN -1.6,MAX 1.9

    VAL_LIMIT(arm_cmd_recv.pitch_arm, -0.8f, 1.0f);
    DMMotorSetRef(pitch_arm, arm_cmd_recv.pitch_arm); // MIN -0.8,MAX 1.0

    PubPushMessage(arm_pub, &arm_feedback_data);
}