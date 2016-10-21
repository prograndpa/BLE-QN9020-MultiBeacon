/**
 ****************************************************************************************
 *
 * @file app_ancsc.h
 *
 * @brief Application ANCSC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_ANCSC_H_
#define APP_ANCSC_H_

/**
 ****************************************************************************************
 * @addtogroup APP_ANCSC_API Apple Notification Center Service Notification Consumer API
 * @ingroup APP_ANCSC
 * @brief Apple Notification Center Service Notification Consumer(NC) API
 *
 * The purpose of the Apple Notification Center Service (ANCS) is to give Bluetooth accessories 
 * a simple and convenient way to access many kinds of notifications that are generated on 
 * iOS devices. It implies it is a GAP Central role. The Profile task for this role will discover
 * the ANCS present on the iOS device, after establishing connection, and will allow configuration
 * of the ANCS attributes if so required. This file contain the implementation of this API. 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

#if BLE_ANCS_NC
#include "ancsc.h"
#include "ancsc_task.h"
#include "app_ancsc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Apple Notification Center Service NC role - at connection
 *
 ****************************************************************************************
 */
void app_ancsc_enable_req(struct ancsc_content *ancs, uint8_t svc_change, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Subscribe to the characteristic with the property notification
 *
 ****************************************************************************************
 */
void app_ancsc_cfg_indntf_req(uint8_t char_code, uint16_t cfg_val, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Get notification attributes command
 * 
 ****************************************************************************************
 */
void app_ancsc_get_ntf_attribute_req(uint32_t notificationUID, uint8_t attribute_size, uint8_t *attribute_list, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Get App attributes command
 *
 ****************************************************************************************
 */
void app_ancsc_get_app_attribute_req(uint16_t list_size, uint8_t *id_list, uint16_t conhdl);

#endif /* BLE_ANCS_NC */

/// @} APP_ANCSC_API

#endif // APP_ANCSC_H_
