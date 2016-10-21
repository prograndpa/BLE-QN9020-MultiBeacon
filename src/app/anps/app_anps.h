/**
 ****************************************************************************************
 *
 * @file app_anps.h
 *
 * @brief Application ANPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_ANPS_H_
#define APP_ANPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_ANPS_API Alert Notification 'Profile' Server
 * @ingroup APP_ANPS
 * @brief Alert Notification 'Profile' Server
 *
 * The Alert Notification service exposes alert information in a device. This information 
 * includes the following: 
 *  - Type of alert occurring in a device 
 *  - Additional text information such as caller ID or sender ID 
 *  - Count of new alerts 
 *  - Count of unread alert items. 
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_AN_SERVER
#include "anps.h"
#include "anps_task.h"
#include "app_anps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Generic message to read a New Alert CFG or Unread Alert Status CFG characteristic
 *
 ****************************************************************************************
 */
void app_anps_create_db(uint16_t supp_new_alert_cat, uint16_t supp_unread_alert_cat);

/*
 ****************************************************************************************
 * @brief Start the Alert Notification Profile server role
 *
 ****************************************************************************************
 */
void app_anps_enable_req(uint8_t conhdl, uint8_t sec_lvl, uint8_t con_type, 
                         uint16_t new_alert_ntf_cfg,
                         uint16_t unread_alert_status_ntf_cfg);

/*
 ****************************************************************************************
 * @brief Update the value of a characteristic
 *
 ****************************************************************************************
 */
void app_anps_ntf_alert_cmd(uint8_t conhdl, uint8_t operation, union anps_value_tag *value);

#endif // BLE_AN_SERVER

/// @} APP_ANPS_API

#endif // APP_ANPS_H_
