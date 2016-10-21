/**
 ****************************************************************************************
 *
 * @file app_pasps.h
 *
 * @brief Application PASPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#ifndef APP_PASPS_H_
#define APP_PASPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_PASPS_API Phone Alert Status Profile Server
 * @ingroup APP_PASPS
 * @brief Phone Alert Status Profile Server
 *
 * The Phone Alert Status Profile can be used to by a phone to expose the current state of
 * the alert proceure. It can be used by peripheral device to configure the way a phone will
 * alert a user.
 * This service has been implemented as a profile. Within this profile, two roles can be 
 * supported: Server role(PASPS) and Client role(PASPC). The Phone Alert Server is the device 
 * that originates the alerts and the Phone Alert Client is the device that receives the alerts
 * and alerts the user. 
 * 
 * @{
 ****************************************************************************************
 */

/// @} APP_PASPS_API

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_PAS_SERVER
#include "pasps.h"
#include "pasps_task.h"
#include "app_pasps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create one instance of the Phone Alert Status Service in the database - at initiation
 *
 ****************************************************************************************
 */
void app_pasps_create_db(uint8_t alert_status, uint8_t ringer_setting);

/*
 ****************************************************************************************
 * @brief Enable the PASP server role for the specified connection - at connection
 *
 ****************************************************************************************
 */
void app_pasps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t alert_status_ntf_cfg, 
                          uint16_t ringer_setting_ntf_cfg);

/*
 ****************************************************************************************
 * @brief Update the value of the Alert Status or the Ringer Setting - at connection
 *
 ****************************************************************************************
 */
void app_pasps_update_char_val_req(uint16_t conhdl, uint8_t operation, uint8_t value);

#endif // BLE_PAS_SERVER

/// @} APP_PASPS_API

#endif // APP_PASPS_H_
