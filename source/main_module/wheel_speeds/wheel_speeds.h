#ifndef _WHEEL_SPEEDS_H
#define _WHEEL_SPEEDS_H

#include <stdbool.h>
#include "main.h"
#include "stm32f407xx.h"

#define TEETH_PER_REVOLUTION (42)
#define WHEEL_DIAMETER_INCHES (16)
#define PI (3.14159F)

bool wheelSpeedsInit();
uint32_t getLeftWheelSpeed();

#endif // _WHEEL_SPEEDS_H
