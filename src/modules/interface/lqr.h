/*
 * lqr.h
 *
 *  Created on: Feb 15, 2017
 *      Author: bitcraze
 */

#ifndef SRC_MODULES_INTERFACE_LQR_H_
#define SRC_MODULES_INTERFACE_LQR_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * phi value is related to attitude.roll
 * theta value is related to attitude.pitch
 * psy value is related to attitude.yaw
 * Parameter values declared here
 * HW_TODO: Are these values to be changed
 * as PARAMETERS, or set values on
 * attitude_pid_controller.c
 */

/*
 * Parameter values
 */

static float k_phi = -30.0;
static float k_theta = -30.0;
static float k_psy = -20.0;
static float k_x = -9.0;
static float k_y = 9.0;
static float k_z = -6.0;
static float C_1 = 1024.0;
static float C_2 = 1/35.0;

float u_opp (float k_p, float attitude, float attitude_ref,
             float k_axis , float w);


float w_1 (float u_x, float u_y, float u_z, float u_t);
float w_2 (float u_x, float u_y, float u_z, float u_t);
float w_3 (float u_x, float u_y, float u_z, float u_t);
float w_4 (float u_x, float u_y, float u_z, float u_t);

#endif /* SRC_MODULES_INTERFACE_LQR_H_ */
