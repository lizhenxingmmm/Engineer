#ifndef SCARA_KINEMATICS_H
#define SCARA_KINEMATICS_H

#include "main.h"
#include <math.h>
#include "VideoTransmitter.h"
#define ARMLENGHT1          220 // unit mm 单位 毫米
#define ARMLENGHT2          220
#define ARMLENGHT3          210
#define MAIN_ARM_ZERO_POINT -0.96
#define MID_ARM_ZERO_POINT  0.43
#define YAW_ZERO_POINT      0.03
#define PITCH_ZERO_POINT    0.66
#define sqrt2               1.414213f

#pragma pack(1)
typedef struct
{
    float init_angle1;
    float init_angle2;
    float init_yaw;
    float init_pitch;
    float init_roll;
    float init_Z;

} initial_state;

#pragma pack()

void scara_inverse_kinematics(float x, float y, float L1, float L2, uint8_t handcoor, float angles[2]);
double polyval_calc(double p[], double x, uint8_t terms);
void check_boundary_scara(float x, float y, float res_xy[2]);
void scara_forward_kinematics(float angle1, float angle2, float L1, float L2, float res_xy[2]);
void GC_get_target_angles_slightly(slightly_controll_data data_pack, float result[6]);
void StateInit(float angle1, float angle2, float angle3, float angle4, float z, float roll_angle);
void RecordMode(uint8_t HeadByte);
void GetCurrentState(float angle1, float angle2, float angle3, float angle4, float z, float roll_angle);
void PushToCube(float result[6], float length);
void check_boundary_scara_lefthand(float x, float y, float res_xy[2]);

#endif
