/*
 * lqr.c
 *
 *  Created on: Feb 15, 2017
 *      Author: bitcraze
 */

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "sensors.h"
#include "math.h"
#include "freefall.h"

float u_opp_x (float k_phi, float roll, float roll_ref,
               float k_x  , float w_x)
{
    float u_x = 0.0;
    return u_x;
}

float u_opp_y (float k_theta, float pitch, float pitch_ref,
               float k_y    , float w_y)
{
    float u_y = 0.0;
    return u_y;
}

float u_opp_z (float k_psy, float yaw, float yaw_ref,
               float k_z  , float w_z)
{
    float u_z = 0.0;
    return u_z;
}
