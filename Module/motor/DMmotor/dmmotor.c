#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"

static uint8_t idx;
static DM_MotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DM_MotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CAN_Instance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff             = motor_can->rx_buff;
    DM_MotorInstance *motor     = (DM_MotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp                    = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position      = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp               = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp             = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DM_MotorInstance *motor = (DM_MotorInstance *)motor_ptr;
    DMMotorEnable(motor);
    DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DMMotorEnable(motor);
    DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
}

void DMMotorCaliEncoder(DM_MotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

static void DMMotorConfigModel(DM_MotorInstance *motor, CAN_Init_Config_s *config)
{
    switch (motor->control_type) {
        case MOTOR_CONTROL_MIT:
            config->tx_id = config->tx_id;
            break;
        case MOTOR_CONTROL_POSITION_AND_SPEED:
            config->tx_id = 0x100 + config->tx_id;
            break;
        case MOTOR_CONTROL_SPEED:
            config->tx_id = 0x200 + config->tx_id;
            break;
        case MOTOR_CONTROL_E_MIT:
            break;
        default:
            break;
    }

    // 检查是否发生id冲突
    for (size_t i = 0; i < idx; i++) {
        if (dm_motor_instance[i]->motor_can_instace->can_handle == config->can_handle &&
            dm_motor_instance[i]->motor_can_instace->tx_id == config->tx_id) {
            uint16_t can_bus __attribute__((unused)) = config->can_handle == &hcan1 ? 1 : 2;
            while (1) // 当控制模式相同且ID相同时,死循环等待
                ;     // 请检查can id是否冲突
        }
    }
}

DM_MotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DM_MotorInstance *motor = (DM_MotorInstance *)malloc(sizeof(DM_MotorInstance));
    memset(motor, 0, sizeof(DM_MotorInstance));

    if (!idx)
        DWT_Delay(1);

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    motor->control_type = config->control_type;
    DMMotorConfigModel(motor, &config->can_init_config);
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id                  = motor;

    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 记得打开
    // 失能，测量数据用
    // DMMotorSetMode(DM_CMD_RESET_MODE, motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DM_MotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorSetSpeedRef(DM_MotorInstance *motor, float ref)
{
    motor->speed_ref = ref;
}

void DMMotorEnable(DM_MotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DM_MotorInstance *motor) // 不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DM_MotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

static void DMMotorMITContoroll(DM_MotorInstance *motor, float ref, DMMotor_Send_s *send)
{
    LIMIT_MIN_MAX(ref, DM_T_MIN, DM_T_MAX);
    send->position_mit = float_to_uint(ref, DM_P_MIN, DM_P_MAX, 16);
    send->velocity_mit = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
    send->torque_des   = float_to_uint(1, DM_T_MIN, DM_T_MAX, 12);
    send->Kp           = 1;
    send->Kd           = 1;

    if (motor->stop_flag == MOTOR_STOP)
        send->torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

    motor->motor_can_instace->tx_buff[0] = (uint8_t)(send->position_mit >> 8);
    motor->motor_can_instace->tx_buff[1] = (uint8_t)(send->position_mit);
    motor->motor_can_instace->tx_buff[2] = (uint8_t)(send->velocity_mit >> 4);
    motor->motor_can_instace->tx_buff[3] = (uint8_t)(((send->velocity_mit & 0xF) << 4) | (send->Kp >> 8));
    motor->motor_can_instace->tx_buff[4] = (uint8_t)(send->Kp);
    motor->motor_can_instace->tx_buff[5] = (uint8_t)(send->Kd >> 4);
    motor->motor_can_instace->tx_buff[6] = (uint8_t)(((send->Kd & 0xF) << 4) | (send->torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] = (uint8_t)(send->torque_des);
}

static void DMMotorPositonSpeedContoroll(DM_MotorInstance *motor, float pos_ref, float speed_ref, DMMotor_Send_s *send)
{

    if (motor->stop_flag == MOTOR_STOP)
        send->velocity_sp = 0;
    else
        send->velocity_sp = speed_ref;
    send->position_sp = pos_ref;
    memcpy(motor->motor_can_instace->tx_buff, &send->position_sp, 4);
    memcpy(motor->motor_can_instace->tx_buff + 4, &send->velocity_sp, 4);
}
//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument)
{
    float pid_ref, speed_ref;
    DM_MotorInstance *motor = (DM_MotorInstance *)argument;
    // DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    // CANInstance *motor_can = motor->motor_can_instace;
    // uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    while (1) {
        pid_ref   = motor->pid_ref;
        speed_ref = motor->speed_ref;

        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
        switch (motor->control_type) {
            case MOTOR_CONTROL_MIT:
                DMMotorMITContoroll(motor, pid_ref, &motor_send_mailbox);
                break;
            case MOTOR_CONTROL_POSITION_AND_SPEED:
                DMMotorPositonSpeedContoroll(motor, pid_ref, speed_ref, &motor_send_mailbox);
                break;
            default:
                break;
        }
        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++) {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}