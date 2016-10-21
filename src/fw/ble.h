/**
 ****************************************************************************************
 *
 * @file ble.h
 *
 * @brief Entry points of the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *
 * @file ble.h
 *
 * @brief NXP revised and cut.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef BLE_H_
#define BLE_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the BLE stack
 *
 * This module contains the primitives that allow an application accessing and running the
 * BLE protocol stack
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief BLE disable core routine
 *
 * This function disables the BLE core.
 ****************************************************************************************
 */
typedef void (*p_ble_disable)(void);
#define ble_disable ((p_ble_disable)(_ble_disable))

/**
****************************************************************************************
* @brief BLE enable core routine
*
* This function enables the BLE core.
****************************************************************************************
*/
typedef void (*p_ble_enable)(void);
#define ble_enable ((p_ble_enable)(_ble_enable))

/**
 ****************************************************************************************
 * @brief Initilaize BLE stack
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
typedef void (*p_qn_ble_init)(void);
#define qn_ble_init ((p_qn_ble_init)(_qn_ble_init))
#endif

/**
 ****************************************************************************************
 * @brief Reset the BLE stack.
 ****************************************************************************************
 */
typedef void (*p_ble_reset)(void);
#define ble_reset ((p_ble_reset)(_ble_reset))

/**
 ****************************************************************************************
 * @brief Gives FW/HW versions of BLE stack.
 *
 ****************************************************************************************
 */
typedef void (*p_ble_version)(uint8_t* fw_version, uint8_t* hw_version);
#define ble_version ((p_ble_version)(_ble_version))

/**
 ****************************************************************************************
 * @brief Run the background processes of the BLE stack
 *
 * This function schedules the internal messages of the BLE protocol stack. It has to be
 * called each time a message is passed from the application to the protocol stack and
 * after an interrupt is generated from the BLE core.
 *
 ****************************************************************************************
 */
typedef void (*p_ble_schedule)(void);
#define ble_schedule ((p_ble_schedule)(_ble_schedule))

/**
 ****************************************************************************************
 * @brief Invoke the sleep function.
 *
 * @return  0 - Disallow sleep, 1 - processor sleep allowed, 2 - BLE sleep allowed
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
int ble_sleep(int usr_sleep_st);
#elif defined(QN_9020_B4)
typedef int (*p_ble_sleep)(int usr_sleep_st);
#define ble_sleep ((p_ble_sleep)(_ble_sleep))
#endif

/**
 ****************************************************************************************
 * @brief BLE interrupt service routine
 *
 * This function is the interrupt service handler of BLE.
 *
 ****************************************************************************************
 */
typedef void (*p_ble_isr)(void);
#define ble_isr ((p_ble_isr)(_ble_isr))

/**
 ****************************************************************************************
 * @brief Register profiles initilization function into BLE stack
 ****************************************************************************************
 */
typedef void (*prf_init_cb)(void);
typedef void (*p_prf_init_reg)(prf_init_cb  profs_init_func);
#define prf_init_reg ((p_prf_init_reg)(_prf_init_reg))

/**
 ****************************************************************************************
 * @brief Register profiles disconnect callback into BLE stack
 ****************************************************************************************
 */
typedef void (*prf_disp_disconnect_cb)(uint8_t status, uint8_t reason, uint16_t conhdl, uint8_t idx);
typedef void (*p_prf_disp_disconnect_reg)(prf_disp_disconnect_cb app_func);
#define prf_disp_disconnect_reg ((p_prf_disp_disconnect_reg)(_prf_disp_disconnect_reg))

/// @} BLE

#endif // BLE_H_
