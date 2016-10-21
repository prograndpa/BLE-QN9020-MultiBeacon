/**
 ****************************************************************************************
 *
 * @file led.c
 *
 * @brief led driver for qn evb.
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
 * @addtogroup  LED
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gpio.h"
#include "led.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   LED initilization
 ****************************************************************************************
 */
void led_init()
{
    // gpio P0.5/P0.4/P0.3/P0.2/P0.1 are output to control led 1~5
    gpio_set_direction_field(LED1_PIN|LED2_PIN|LED3_PIN|LED4_PIN|LED5_PIN, (uint32_t)GPIO_OUTPUT);

    // all led are off
    // led_matrix(0x0); 
    led_set(1, LED_OFF);
    led_set(2, LED_OFF);
    led_set(3, LED_OFF);
    led_set(4, LED_OFF);
    led_set(5, LED_OFF);
}

/**
 ****************************************************************************************
 * @brief   Control led matrix
 * @param[in]    matrix     bit0~bit4 -> led 1~5 (1-on, 0-off)
 * @description
 *  This function controls all of the leds at the same time.
 ****************************************************************************************
 */
void led_matrix(uint32_t matrix)
{
    uint32_t reg = 0;    
    
    matrix = ~matrix;
    reg = ((matrix & 0x01) << 4) | ((matrix & 0x02) << 2) | (matrix & 0x04) | ((matrix & 0x08) >> 2) | ((matrix & 0x10) >> 4);  
    gpio_write_pin_field(LED1_PIN|LED2_PIN|LED3_PIN|LED4_PIN|LED5_PIN, reg<<1);
}

/**
 ****************************************************************************************
 * @brief   Set led on/off
 * @param[in]    idx    1~5 -> led 1~5
 * @param[in]    enable on/off
 * @description
 *  This function switchs on/off led individually.
 ****************************************************************************************
 */
void led_set(uint32_t idx, enum led_st enable)
{
    enum gpio_pin reg;
    
    switch(idx)
    {
        case 1:
            reg = LED1_PIN;
            break;
        case 2:
            reg = LED2_PIN;
            break;
        case 3:
            reg = LED3_PIN;
            break;
        case 4:
            reg = LED4_PIN;
            break;
        case 5:
            reg = LED5_PIN;
            break;
        default:
            return;
    }

    gpio_write_pin(reg, (enum gpio_level)enable);
}

/**
 ****************************************************************************************
 * @brief   Get led status
 * @param[in]    idx    1~5 -> led 1~5
 * @return       led on/off
 * @description
 *  This function get led status individually.
 ****************************************************************************************
 */
enum led_st led_get(uint32_t idx)
{
    enum gpio_pin reg;
    
    switch(idx)
    {
        case 1:
            reg = LED1_PIN;
            break;
        case 2:
            reg = LED2_PIN;
            break;
        case 3:
            reg = LED3_PIN;
            break;
        case 4:
            reg = LED4_PIN;
            break;
        case 5:
            reg = LED5_PIN;
            break;
        default:
            return LED_OFF;
    }

    return (enum led_st)gpio_read_pin(reg);
}

/// @} LED



