#include "arm.h"
#include "dmmotor.h"

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
    // 机械臂控制任务
    // DMMotorSetRef(maximal_arm, -0.4); // MIN -1.5,MAX 0.4
    // DMMotorSetSpeedRef(maximal_arm, 0.5);

    // DMMotorSetRef(minimal_arm, 0.5); // MIN -2.4,MAX 2.7
    // DMMotorSetSpeedRef(minimal_arm, 0.5);

    // DMMotorSetRef(finesse, 0); // MIN -2,MAX 3.3
    // DMMotorSetSpeedRef(finesse, 0.5);

    // DMMotorSetRef(pitch_arm, 0.5); // MIN -0.8,MAX 0.6
    // DMMotorSetSpeedRef(pitch_arm, 0.5);
}