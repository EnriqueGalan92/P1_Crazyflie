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
#define ZERO 0.0f

float u_opp (float k_p, float attitude, float attitude_ref,
             float k_axis , float w);

float w_1 (float u_x, float u_y, float u_z, float u_t, float C_1, float C_2);
float w_2 (float u_x, float u_y, float u_z, float u_t, float C_1, float C_2);
float w_3 (float u_x, float u_y, float u_z, float u_t, float C_1, float C_2);
float w_4 (float u_x, float u_y, float u_z, float u_t, float C_1, float C_2);

#endif /* SRC_MODULES_INTERFACE_LQR_H_ */
