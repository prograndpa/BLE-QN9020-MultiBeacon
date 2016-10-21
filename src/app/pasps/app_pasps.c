/**
 ****************************************************************************************
 *
 * @file app_pasps.c
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

/**
 ****************************************************************************************
 * @addtogroup APP_PASPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_PAS_SERVER
#include "app_pasps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create one instance of the Phone Alert Status Service in the database - at initiation *//**
 *
 * @param[in] alert_status The Alert Status characteristic returns the current value of 
 * server¡¯s alert status when read. Possible values are:
 * - PASP_RINGER_ACTIVE
 * - PASP_VIBRATE_ACTIVE
 * - PASP_DISP_ALERT_STATUS_ACTIVE
 * @param[in] ringer_setting The Ringer Setting characteristic exposes the state of the 
 * server device's ringer. Possible values are:
 * - PASP_RINGER_SILENT
 * - PASP_RINGER_NORMAL
 *
 * @response PASPS_CMP_EVT
 * @description
 * This function shall be called after system power-on in order to create Phone Alert Status
 * Service. This database will be visible from a peer device but not usable until this
 * service enabled within a BLE connection.
 *
 ****************************************************************************************
 */
void app_pasps_create_db(uint8_t alert_status, uint8_t ringer_setting)
{
    struct pasps_create_db_req * msg = KE_MSG_ALLOC(PASPS_CREATE_DB_REQ, TASK_PASPS, TASK_APP, pasps_create_db_req);

    msg->alert_status = alert_status;
    msg->ringer_setting = ringer_setting;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Enable the PASP server role for the specified connection - at connection *//**
 *
 * @param[in] conhdl Connection handle for which the PASP Service is enabled
 * @param[in] sec_lvl Security level required for protection of PASPS attributes
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(PRF_CON_DISCOVERY) or normal(PRF_CON_NORMAL)
 * - Normal connection: Peer device is known(bonded) and Client Configuration Characteristics
 *   values shall be restored
 * - Discovery connection: Peer device is unknown and peer collector will manage Client Configuration 
 *   Characteristics
 * @param[in] alert_status_ntf_cfg Stored Alert Status notification configuration for a bonded device
 * @param[in] ringer_setting_ntf_cfg Stored Ringer Setting notification configuration for a bonded device
 *
 * @response PASPS_CMP_EVT
 * @description
 * This function shall be called after the connection with a peer device has been established
 * in order to enable the PASP server role for the specified connection.
 *
 ****************************************************************************************
 */
void app_pasps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t alert_status_ntf_cfg, 
                          uint16_t ringer_setting_ntf_cfg)
{
    struct pasps_enable_req * msg = KE_MSG_ALLOC(PASPS_ENABLE_REQ, KE_BUILD_ID(TASK_PASPS, conhdl), TASK_APP, pasps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->alert_status_ntf_cfg = alert_status_ntf_cfg;
    msg->ringer_setting_ntf_cfg = ringer_setting_ntf_cfg;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Update the value of the Alert Status or the Ringer Setting - at connection *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] operation Operation code, possible values are:
 * - PASPS_UPD_ALERT_STATUS_OP_CODE
 * - PASPS_UPD_RINGER_SETTING_OP_CODE
 * @param[in] value Alert Status value or Ringer Setting value
 *
 * @response PASPS_CMP_EVT
 * @description
 * This function is used to update the value of the Alert Status or the Ringer Setting.
 *
 ****************************************************************************************
 */
void app_pasps_update_char_val_req(uint16_t conhdl, uint8_t operation, uint8_t value)
{
    struct pasps_update_char_val_cmd * msg = KE_MSG_ALLOC(PASPS_UPDATE_CHAR_VAL_CMD, KE_BUILD_ID(TASK_PASPS, conhdl), TASK_APP, pasps_update_char_val_cmd);

    msg->conhdl = conhdl;
    msg->operation = operation;
    msg->value = value;
    ke_msg_send(msg);
}

#endif // BLE_PAS_SERVER

/// @} APP_PASPS_API
