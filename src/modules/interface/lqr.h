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
 * #define k_phi -30.0
 * #define k_theta -30.0
 * #define k_psy -20.0
 * #define k_x -9.0
 * #define k_y +9.0
 * #define k_z -6.0
 * #define C_1 1024.0
 * #define C_2 1/35.0
 */
float u_opp_x (float k_phi, float roll, float roll_ref,
               float k_x  , float w_x);

float u_opp_y (float k_theta, float pitch, float pitch_ref,
               float k_y    , float w_y);

float u_opp_z (float k_psy, float yaw, float yaw_ref,
               float k_z  , float w_z);

#endif /* SRC_MODULES_INTERFACE_LQR_H_ */
