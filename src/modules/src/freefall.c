/*
 * freefall.c
 *
 *  Created on: Feb 10, 2017
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

float acc_mag(float x, float y, float z)
{
    float xx, yy, zz;
    xx = x*x;
    yy = y*y;
    zz = z*z;
    return sqrt(xx+yy+zz);
}
bool detect_freefall (float acc)
{
    if (acc >= 0.0f && acc <= 0.1f )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
