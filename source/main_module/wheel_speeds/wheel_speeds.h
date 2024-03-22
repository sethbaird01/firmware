#ifndef _WHEEL_SPEEDS_H
#define _WHEEL_SPEEDS_H

#include <stdbool.h>
#include "main.h"
#include "stm32f407xx.h"

#define TEETH_PER_REVOLUTION 42

bool wheelSpeedsInit();
uint32_t getLeftWheelSpeed();

#endif // _WHEEL_SPEEDS_H
