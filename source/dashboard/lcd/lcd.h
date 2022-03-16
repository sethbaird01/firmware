#ifndef __LCD_H__
#define __LCD_H__

#include "common/phal_L4/spi/spi.h"
#include "nextion.h"
#include "pedals.h"
#include "can_parse.h"
#include "main.h"

#define BTN_NORM_ID 4
#define BTN_HIGH_ID 11
#define BTN_SELECT_TIMEOUT_MS 5000

typedef enum
{
    P_SPLASH,
    P_MAIN,
    P_SETTINGS,
    P_INFO,
    P_TOTAL
} pages_t;

typedef enum
{
    B_TC_BUTTON,
    B_DIAG_BUTTON,
    B_LAPS_BUTTON,
    B_START_BUTTON,
    B_SETTINGS,
    B_MAIN_TOTAL
} main_buttons_t;

typedef enum
{
    A_SPEED,
    A_VOLTAGE,
    A_BATTERY_BAR,
    A_STATUS_LABEL,
    A_MAIN_TOTAL
} main_attributes_t;

typedef enum
{
    B_BACK_BUTTON,
    B_INFO_TOTAL
} info_buttons_t;

typedef enum
{
    B_BACK_BUTTON_S,
    B_SETTINGS_TOTAL
} settings_buttons_t;

void joystickUpdatePeriodic();
void changePage(uint8_t new_page);

#endif