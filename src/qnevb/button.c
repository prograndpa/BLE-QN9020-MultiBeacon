/**
 ****************************************************************************************
 *
 * @file button.c
 *
 * @brief button driver for qn evb.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  BUTTON
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "button.h"

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Button initilization
 ****************************************************************************************
 */
void button_init(void)
{
    // button 1
    //gpio_pull_set(BUTTON1_PIN, GPIO_PULL_UP);
    //gpio_set_direction_field(BUTTON1_PIN, (uint32_t)GPIO_INPUT);
    //gpio_set_interrupt(BUTTON1_PIN, GPIO_INT_FALLING_EDGE);
    gpio_wakeup_config(BUTTON1_PIN, GPIO_WKUP_BY_LOW);
    gpio_enable_interrupt(BUTTON1_PIN);

    // button 2
    //gpio_pull_set(BUTTON2_PIN, GPIO_PULL_UP);
    //gpio_set_direction_field(BUTTON2_PIN, (uint32_t)GPIO_INPUT);
    //gpio_set_interrupt(BUTTON2_PIN, GPIO_INT_FALLING_EDGE);
    gpio_wakeup_config(BUTTON2_PIN, GPIO_WKUP_BY_LOW);
    gpio_enable_interrupt(BUTTON2_PIN);
}

/**
 ****************************************************************************************
 * @brief   check button state: 0: press, 1: release
 ****************************************************************************************
 */
int check_button_state(int btn)
{
    if (gpio_read_pin_field(btn) == btn) {
        return 1;
    }
    else {
        return 0;
    }
}

/// @} BUTTON

