/**
 ****************************************************************************************
 *
 * @file app_scpps.c
 *
 * @brief Application SCPPS API
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
 * @addtogroup APP_SCPPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"
#if BLE_SP_SERVER
#include "app_scpps.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the scan parameters profile server database - at initiation     *//**
 * 
 * @param[in] features Indicate if scan fresh function is supported or not, possible values are:
 * - SCPPS_SCAN_REFRESH_CHAR_NOT_SUP
 * - SCPPS_SCAN_REFRESH_CHAR_SUP
 *
 * @response SCPPS_CREATE_DB_CFM
 * @description
 * This function is used to add one instance of the Scan Parameters Service in the database.
 ****************************************************************************************
 */
void app_scpps_create_db(uint8_t features)
{
    struct scpps_create_db_req * msg = KE_MSG_ALLOC(SCPPS_CREATE_DB_REQ, TASK_SCPPS, TASK_APP, scpps_create_db_req);

    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the scan parameters profile server service - at connection      *//**
 *
 * @param[in] conhdl Connection handle for which the profile scan parameters server role is enabled.
 * @param[in] sec_lvl Security level required for protection of SCPS attributes:
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * @param[in] scan_refresh_ntf_en Scan Refresh Notification Configurations, possible values are:
 * - PRF_CLI_STOP_NTFIND
 * - PRF_CLI_START_NTF
 *
 * @response None or SCPPS_EORROR_IND
 * @description
 * This function shall be used after the connection with a peer device has been established
 * in order to enable the SCPP Server role task for the specified connection.
 ****************************************************************************************
 */
void app_scpps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t scan_refresh_ntf_en)
{
    struct scpps_enable_req * msg = KE_MSG_ALLOC(SCPPS_ENABLE_REQ, TASK_SCPPS, TASK_APP, scpps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->scan_refresh_ntf_en = scan_refresh_ntf_en;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send the scan refresh value - at connection       *//**
 *
 * @param[in] conhdl Connection handle for which the profile scan parameters server role is enabled.
 * @param[in] scan_refresh Scan Refresh Value, 0 means Server requires refresh, 1 ~ 255
 * reserved for future use
 *
 * @response SCPPS_SCAN_REFRESH_SEND_CFM
 * @description
 * This function is used notify the Client that the Server writes the latest intended scan
 * parameters to the Scan Interval Window Characteristic.
 ****************************************************************************************
 */
void app_scpps_scan_refresh_req(uint16_t conhdl, uint8_t scan_refresh)
{
    struct scpps_scan_refresh_send_req * msg = KE_MSG_ALLOC(SCPPS_SCAN_REFRESH_SEND_REQ, TASK_SCPPS, TASK_APP, scpps_scan_refresh_send_req);
    
    msg->conhdl = conhdl;
    msg->scan_refresh = scan_refresh;
    ke_msg_send(msg);
}

#endif // BLE_SP_SERVER

/// @} APP_SCPPS_API
