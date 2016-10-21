
/**
 ****************************************************************************************
 *
 * @file rng.h
 *
 * @brief Header file of RNG for QN9020.
 *
 * Copyright(C) 2016 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _RNG_H_
#define _RNG_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_RNG==TRUE

/**
 ****************************************************************************************
 * @defgroup RNG RNG Driver
 * @ingroup DRIVERS
 * @brief Random number generator driver
 *
 *  The purpose of Watchdog Timer (WDT) is to perform a system reset after the software
 *  running into a problem. This prevents system from hanging for an infinite period of time.
 *  The main features of QN9020 WDT are listed as follow:
 *    - 32-bit down counter with a programmable timeout interval
 *    - 32KHz clock(WDOGCLK=PCLK, WDOGCLKEN=32K)
 *    - Interrupt output generation on timeout
 *    - Reset signal generation on timeout if the interrupt from the previous timeout remains unserviced by software
 *    - Lock register to protect registers from being altered by runaway software
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Get a random number.
 *****************************************************************************************
 */
uint32_t rng_get(void);


/// @} RNG
#endif /* CONFIG_ENABLE_DRIVER_RNG==TRUE */
#endif /* _RNG_H_ */
