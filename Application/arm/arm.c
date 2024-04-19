#include "arm.h"
#include "robot_def.h"

#include "dmmotor.h"
#include "DJI_motor.h"
#include "message_center.h"
#include "user_lib.h"
#include "tim.h"
#include "bsp_dwt.h"
// 大臂 -0.19 小臂 -0.79就寄啦？
#define LIFT_OFFSET (-287.81269f)
#define ROLL_OFFSET 35.07f

// 图传链路 大臂零位maximal_arm -0.96 小臂零位minimal_arm 0.43 手腕零位 finesse 0.03 pitch_arm 零位 0.66
static Publisher_t *arm_pub;  // 用于发布底盘的数据
static Subscriber_t *arm_sub; // 用于订阅底盘的控制命令

static Arm_Ctrl_Cmd_s arm_cmd_recv;         // 发送给底盘的控制命令
static Arm_Upload_Data_s arm_feedback_data; // 从底盘接收的数据

static DM_MotorInstance *maximal_arm, *minimal_arm, *finesse, *pitch_arm;
static DJIMotor_Instance *lift, *roll;
static int8_t is_init;
static float roll_init_angle, roll_real, lift_init_angle, height, lift_speed_feedfoward = 5.0f, lift_current_feedfoward = 1.f;

static void ArmDMInit(void) // 非常抽象的函数，达妙电机不给值会回到原位，当然可以重新置零，但是工程的机械臂要限位？
{
    DMMotorSetRef(maximal_arm, maximal_arm->measure.position);
    DMMotorSetSpeedRef(maximal_arm, 0.3);

    DMMotorSetRef(minimal_arm, minimal_arm->measure.position);
    DMMotorSetSpeedRef(minimal_arm, 1);

    DMMotorSetRef(finesse, finesse->measure.position);
    DMMotorSetSpeedRef(finesse, 1.5);

    DMMotorSetRef(pitch_arm, pitch_arm->measure.position);
    DMMotorSetSpeedRef(pitch_arm, 1.5);

    arm_sub = SubRegister("arm_cmd", sizeof(Arm_Ctrl_Cmd_s));
    arm_pub = PubRegister("arm_feed", sizeof(Arm_Upload_Data_s));
}

static void Height_Calculation(void)
{
    height = -(lift->measure.total_angle - lift_init_angle) / LIFT_OFFSET;
}

static void Roll_Calculation(void)
{
    roll_real = (roll->measure.total_angle - roll_init_angle) / ROLL_OFFSET;
}

static void Sucker_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //?????????1??PWM???
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    DWT_Delay(2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
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
        .controller_param_init_config.dm_mit_PID = {
            .Kp = 20, // 20
            .Kd = 5,  // 达妙mit模式下的PID不需要Ki，千万不要kp = 0 && kd = 0
        },
        // 速度位置模式下不需要PID,喵老板真棒^^
        // .control_type = MOTOR_CONTROL_POSITION_AND_SPEED,
        .control_type = MOTOR_CONTROL_MIT,
        .motor_type   = DM8006,
    };
    maximal_arm = DMMotorInit(&motor_config);

    motor_config.can_init_config.rx_id = 0x04;
    motor_config.can_init_config.tx_id = 2;
    motor_config.control_type          = MOTOR_CONTROL_POSITION_AND_SPEED;
    minimal_arm                        = DMMotorInit(&motor_config);

    motor_config.can_init_config.can_handle = &hcan2;
    motor_config.can_init_config.rx_id      = 0x03;
    motor_config.can_init_config.tx_id      = 1;
    finesse                                 = DMMotorInit(&motor_config);

    motor_config.can_init_config.rx_id = 0x04;
    motor_config.can_init_config.tx_id = 2;
    pitch_arm                          = DMMotorInit(&motor_config);

    Motor_Init_Config_s lift_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 200,
                .Ki                = 1,
                .Kd                = 2,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .IntegralLimit     = 10000,
                .MaxOut            = 15000,
                .Derivative_LPF_RC = 0.01,
            },
            .speed_PID = {
                .Kp = 1,  // 10
                .Ki = 75, // 1
                .Kd = 0,
                // .CoefA         = 0.2,
                // .CoefB         = 0.3,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
            .current_PID = {
                .Kp            = 0.9, // 0.7
                .Ki            = 0.1, // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
                // .DeadBand      = 0.1,
            },
            .other_angle_feedback_ptr = &height,
            .speed_feedforward_ptr    = &lift_speed_feedfoward,
            .current_feedforward_ptr  = &lift_current_feedfoward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, // ！！！ 只有在只对速度闭环时才能反向 ！！！
            .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
        },
        .motor_type = M3508};
    lift = DJIMotorInit(&lift_config);

    Motor_Init_Config_s roll_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 10,
                .Ki                = 0,
                .Kd                = 0,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .IntegralLimit     = 10000,
                .MaxOut            = 15000,
                .Derivative_LPF_RC = 0.01,
            },
            .speed_PID = {
                .Kp            = 2,   // 10
                .Ki            = 0.1, // 1
                .Kd            = 0.002,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
            .current_PID = {
                .Kp            = 1.5,  // 0.7
                .Ki            = 0.02, // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
            .other_angle_feedback_ptr = &roll_real,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // ！！！ 只有在只对速度闭环时才能反向 ！！！
        },
        .motor_type = M2006};
    roll = DJIMotorInit(&roll_config);
    HAL_TIM_Base_Start(&htim1); // 开启定时器1
    Sucker_Init();
}

// lift的机械距离大约350mm 最高机械角度-64948 最低机械角度 22115
// 机械行程 87063度
// 读角度时init角度应位于最高点 init角度对应350mm 最低点为init角度-87063 对应0mm
// (48733.0234 + 195.158646) / 17.00 = OFFSET

// roll init 2066.6
// 360度后 14692.2
// roll_offset = (roll - roll_init) * 360 =
void ARMTask(void)
{
    if (!is_init) {
        ArmDMInit();
        roll_init_angle = roll->measure.total_angle; // min = -3460 - 165 max =4973 - 165
        lift_init_angle = lift->measure.total_angle;
        is_init         = 1;
    }
    SubGetMessage(arm_sub, &arm_cmd_recv);
    // 机械臂控制任务
    if (arm_cmd_recv.arm_mode == ARM_ZERO_FORCE) {
        DMMotorStop(maximal_arm);
        DMMotorStop(minimal_arm);
        DMMotorStop(finesse);
        DMMotorStop(pitch_arm);
        DJIMotorStop(lift);
        DJIMotorStop(roll);
    } else {
        DMMotorEnable(maximal_arm);
        DMMotorEnable(minimal_arm);
        DMMotorEnable(finesse);
        DMMotorEnable(pitch_arm);
        DJIMotorEnable(lift);
        DJIMotorEnable(roll);
    }

    VAL_LIMIT(arm_cmd_recv.maximal_arm, MAXARM_MIN, MAXARM_MAX);
    VAL_LIMIT(arm_cmd_recv.minimal_arm, MINARM_MIN, MINARM_MAX);
    VAL_LIMIT(arm_cmd_recv.finesse, FINE_MIN, FINE_MAX);
    VAL_LIMIT(arm_cmd_recv.pitch_arm, PITCH_MIN, PITCH_MAX);
    VAL_LIMIT(arm_cmd_recv.lift, HEIGHT_MIN, HEIGHT_MAX);
    DMMotorSetRef(maximal_arm, arm_cmd_recv.maximal_arm); // MIN -1.0,MAX 0.75
    DMMotorSetRef(minimal_arm, arm_cmd_recv.minimal_arm); // MIN -2.0,MAX 2.7
    DMMotorSetRef(finesse, arm_cmd_recv.finesse);         // MIN -1.6,MAX 1.9
    DMMotorSetRef(pitch_arm, arm_cmd_recv.pitch_arm);     // MIN -0.8,MAX 1.0

    if (arm_cmd_recv.up_flag != 0) {
        DJIMotorOuterLoop(lift, ANGLE_LOOP);
        DJIMotorSetRef(lift, arm_cmd_recv.lift);
    } else {
        DJIMotorOuterLoop(lift, ANGLE_LOOP);
        DJIMotorSetRef(lift, height);
    }

    if (arm_cmd_recv.roll_flag != 0) {
        // DJIMotorOuterLoop(roll, ANGLE_LOOP);
        // DJIMotorSetRef(roll, arm_cmd_recv.roll);
        if (arm_cmd_recv.roll - roll_real > 2) {
            DJIMotorSetRef(roll, 2500);
        } else if (arm_cmd_recv.roll - roll_real < -2) {
            DJIMotorSetRef(roll, -2500);
        } else {
            DJIMotorSetRef(roll, 0);
        }
        // DJIMotorSetRef(roll, 2500 * arm_cmd_recv.roll_flag);
    } else {
        // DJIMotorOuterLoop(roll, ANGLE_LOOP);
        // DJIMotorSetRef(roll, roll_real);
        DJIMotorSetRef(roll, 0);
    }

    if (arm_cmd_recv.sucker_flag) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
    } else {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
    }

    if (arm_cmd_recv.dm_state == DM_MOTOR_ERR) {
        // DMMotorClearErr(maximal_arm);
        // DMMotorClearErr(minimal_arm);
        // DMMotorClearErr(finesse);
        // DMMotorClearErr(pitch_arm);
    }

    Height_Calculation();
    Roll_Calculation();

    arm_feedback_data.maximal_arm = maximal_arm->measure.position;
    arm_feedback_data.minimal_arm = minimal_arm->measure.position;
    arm_feedback_data.finesse     = finesse->measure.position;
    arm_feedback_data.pitch_arm   = pitch_arm->measure.position;
    arm_feedback_data.height      = height;
    arm_feedback_data.roll        = roll_real;
    PubPushMessage(arm_pub, &arm_feedback_data);
}