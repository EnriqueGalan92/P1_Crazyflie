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
#include "lqr.h"


float u_opp (float k_p, float attitude, float attitude_ref,
             float k_axis , float w)
{
    float u_res = 0.0;
    return u_res;
}

float w_1 (float u_x, float u_y, float u_z, float u_t)
{
    return C_1*sqrt(-u_x + u_y - u_z + (C_2*u_t));
}

float w_2 (float u_x, float u_y, float u_z, float u_t)
{
    return C_1*sqrt(-u_x - u_y + u_z + (C_2*u_t));
}

float w_3 (float u_x, float u_y, float u_z, float u_t)
{
    return C_1*sqrt( u_x - u_y - u_z + (C_2*u_t));
}

float w_4 (float u_x, float u_y, float u_z, float u_t)
{
    return C_1*sqrt( u_x + u_y + u_z + (C_2*u_t));
}
