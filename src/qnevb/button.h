/**
 ****************************************************************************************
 *
 * @file button.h
 *
 * @brief Header file of button driver for qn evb.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _BUTTON_H_
#define _BUTTON_H_

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
#if defined(QN_9021_MINIDK)

#if defined(CFG_FCC_CE_TEST)
#define BUTTON1_PIN    (CFG_FCC_CE_CTRL_PIN)
#else
#define BUTTON1_PIN    (GPIO_P12)
#endif
#define BUTTON2_PIN    (GPIO_P10)

#elif defined(QN_9022_MINIDK_V1_1)

#if defined(CFG_FCC_CE_TEST)
#define BUTTON1_PIN    (CFG_FCC_CE_CTRL_PIN)
#else
#define BUTTON1_PIN    (GPIO_P03)
#endif
#define BUTTON2_PIN    (GPIO_P16)

#else   // QN9020 MINIDK

#if defined(CFG_FCC_CE_TEST)
#define BUTTON1_PIN    (CFG_FCC_CE_CTRL_PIN)
#else
#define BUTTON1_PIN    (GPIO_P14)
#endif
#define BUTTON2_PIN    (GPIO_P15)

#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

extern void button_init(void);
extern int check_button_state(int btn);

#endif

