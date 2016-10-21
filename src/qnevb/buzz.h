/**
 ****************************************************************************************
 *
 * @file buzz.h
 *
 * @brief Header file of buzz driver for qn evb.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _BUZZ_H_
#define _BUZZ_H_

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define BUZZ_PWM_PERIOD 200

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

enum buzz_vol
{
    BUZZ_VOL_HIGH = 100,
    BUZZ_VOL_LOW = 20
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

extern void buzzer_on(enum buzz_vol volume);
extern void buzzer_off(void);


#endif
