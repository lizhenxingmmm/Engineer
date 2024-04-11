#ifndef SCARA_KINEMATICS_H
#define SCARA_KINEMATICS_H

#include "main.h"
#include <math.h>
#define ARMLENGHT1 220 // unit mm 单位 毫米
#define ARMLENGHT2 220


void scara_inverse_kinematics(float x, float y, float L1, float L2, uint8_t handcoor, float angles[2]);
double polyval_calc(double p[], double x, uint8_t terms);
void check_boundary_scara(float x, float y, float res_xy[2]);
void scara_forward_kinematics(float angle1, float angle2, float L1, float L2, float res_xy[2]);
void GC_get_target_angles_slightly(float angles[4], float z, float roll_angle, float delta_x, float delta_y, float delta_z, float delta_yaw, float delta_pitch, float delta_roll, float result[6]);

#endif
