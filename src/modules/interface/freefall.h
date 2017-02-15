/*
 * freefall.h
 *
 *  Created on: Feb 10, 2017
 *      Author: bitcraze
 */

#ifndef SRC_MODULES_INTERFACE_FREEFALL_H_
#define SRC_MODULES_INTERFACE_FREEFALL_H_

#include <stdbool.h>
#include <stdint.h>

#define FALSE 0
#define TRUE 1

float acc_mag(float x, float y, float z);
bool detect_freefall (float acc);

#endif /* SRC_MODULES_INTERFACE_FREEFALL_H_ */
