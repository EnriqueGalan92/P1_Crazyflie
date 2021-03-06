/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "ext_position.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
#include "motors.h"
#include "freefall.h"
#include "lqr.h"
#include "crtp.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static float test_var = 0;
static float acc_var = 0;
static bool freefalling = FALSE;

static float ux = 0.0;
static float uy = 0.0;
static float uz = 0.0;
static float w1 = 0.0;
static float w2 = 0.0;
static float w3 = 0.0;
static float w4 = 0.0;

static float k_phi = -30.0;
static float k_theta = -30.0;
static float k_psy = -20.0;
static float k_x = -9.0;
static float k_y = 9.0;
static float k_z = -6.0;
static float C_1 = 1024.0;
static float C_2 = 1/35.0;

static uint16_t w1_motor = 0;
static uint16_t w2_motor = 0;
static uint16_t w3_motor = 0;
static uint16_t w4_motor = 0;

const uint16_t MAX_MOTORS = 65534;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  static uint32_t tick_t = 0;
  static uint32_t dta_Tick = 0;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    test_var += 0.1f;
    acc_var = acc_mag(sensorData.acc.x,sensorData.acc.y, sensorData.acc.z);
    freefalling = detect_freefall(acc_var);
/*
 * Part of the freefalling activity
    if (freefalling)
    {
        motorsSetRatio(MOTOR_M1, 30000);
        motorsSetRatio(MOTOR_M2, 30000);
        motorsSetRatio(MOTOR_M3, 30000);
        motorsSetRatio(MOTOR_M4, 30000);
    }
    else
    {
        motorsSetRatio(MOTOR_M1, 0);
        motorsSetRatio(MOTOR_M2, 0);
        motorsSetRatio(MOTOR_M3, 0);
        motorsSetRatio(MOTOR_M4, 0);
    }
*/
    // state gets information, lqr to be used after this
    getExtPosition(&state);

#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorUpdate(&state, &sensorData, &control);
#else
    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, tick);
#endif

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    ux = u_opp(k_phi, state.attitude.roll, setpoint.attitude.roll,
               k_x, sensorData.gyro.x);
    uy = u_opp(k_theta, state.attitude.pitch, setpoint.attitude.pitch,
               k_y, sensorData.gyro.y);
    uz = u_opp(k_psy, state.attitude.yaw, setpoint.attitude.yaw,
               k_z, sensorData.gyro.z);

    w1 = w_1(ux,uy,uz,setpoint.thrust, C_1, C_2);
    w2 = w_2(ux,uy,uz,setpoint.thrust, C_1, C_2);
    w3 = w_3(ux,uy,uz,setpoint.thrust, C_1, C_2);
    w4 = w_4(ux,uy,uz,setpoint.thrust, C_1, C_2);

    if (MAX_MOTORS > (uint16_t)w1)
        w1_motor = (uint16_t)w1;
    else
        w1_motor = MAX_MOTORS;

    if (MAX_MOTORS > (uint16_t)w2)
        w2_motor = (uint16_t)w2;
    else
        w2_motor = MAX_MOTORS;

    if (MAX_MOTORS > (uint16_t)w3)
        w3_motor = (uint16_t)w3;
    else
        w3_motor = MAX_MOTORS;

    if (MAX_MOTORS > (uint16_t)w4)
        w4_motor = (uint16_t)w4;
    else
        w4_motor = MAX_MOTORS;

    tick_t = xTaskGetTickCount();
    dta_Tick = tick_t - get_last_Tick();
    if ( dta_Tick < 1000)
    {
        motorsSetRatio(MOTOR_M1, w1_motor);
        motorsSetRatio(MOTOR_M2, w2_motor);
        motorsSetRatio(MOTOR_M3, w3_motor);
        motorsSetRatio(MOTOR_M4, w4_motor);
    }
    else
    {
        motorsSetRatio(MOTOR_M1, 0);
        motorsSetRatio(MOTOR_M2, 0);
        motorsSetRatio(MOTOR_M3, 0);
        motorsSetRatio(MOTOR_M4, 0);
    }

    //stateController(&control, &setpoint, &sensorData, &state, tick);
    //powerDistribution(&control);

    tick++;
  }
}


LOG_GROUP_START(freefall_group)
LOG_ADD(LOG_FLOAT, acc_mag, &acc_var)
LOG_ADD(LOG_UINT16, freefalling, &freefalling)
LOG_GROUP_STOP(freefall_group)

LOG_GROUP_START(control_group)
LOG_ADD(LOG_FLOAT, w1, &w1)
LOG_ADD(LOG_FLOAT, w2, &w2)
LOG_ADD(LOG_FLOAT, w3, &w3)
LOG_ADD(LOG_FLOAT, w4, &w4)
LOG_ADD(LOG_UINT16, w1_motors, &w1_motor)
LOG_ADD(LOG_UINT16, w2_motors, &w2_motor)
LOG_ADD(LOG_UINT16, w3_motors, &w3_motor)
LOG_ADD(LOG_UINT16, w4_motors, &w4_motor)
LOG_ADD(LOG_FLOAT, ux, &ux)
LOG_ADD(LOG_FLOAT, uy, &uy)
LOG_ADD(LOG_FLOAT, uz, &uz)
LOG_GROUP_STOP(control_group)

LOG_GROUP_START(test_group)
LOG_ADD(LOG_FLOAT, test, &test_var)
LOG_GROUP_STOP(test_group)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)
