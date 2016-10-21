/**
 ****************************************************************************************
 *
 * @file app_scppc.c
 *
 * @brief Application SCPPC API
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
 * @addtogroup APP_SCPPC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_SP_CLIENT
#include "app_scppc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Scan Parameters profile Client Role - at connection. *//**
 * @param[in] scan_intv     Last Scan Interval value to write after discovery.
 * @param[in] scan_window   Last Scan Window value to write after discovery.
 * @param[in] scps          Scan Parameters Service Content Structure.
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  SCPPC_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Client role of the SCPP. This Function contains BLE 
 *  connection handle, the connection type and the previously saved discovered SCPS details on peer. 
 *  The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration or PRF_CON_NORMAL 
 *  (0x01) for a normal connection with a bonded device. Application shall save those information to reuse them for 
 *  other connections. During normal connection, previously discovered device information can be reused. 
 *  If it is a discovery/configuration type of connection, it is useless to fill the scps parameter are useless. Otherwise they 
 *  will contain pertinent data which will be kept in the Client environment while enabled. 
 *
 *  For a normal connection, the response to this request is sent right away after saving the SCPS content in the 
 *  environment and registering SCPPC in GATT to receive the notifications for the known attribute handles in SCPS that 
 *  would be notified (Scan Refresh Characteristic). 
 *
 *  For a discovery connection, discovery of the peer SCPS is started and the response will be sent at the end of the 
 *  discovery with the discovered attribute details.
 *
 *****************************************************************************************
 */
void app_scppc_enable_req(uint16_t scan_intv, uint16_t scan_window, struct scps_content *scps, uint16_t conhdl)
{
    struct scppc_enable_req * msg = KE_MSG_ALLOC(SCPPC_ENABLE_REQ, KE_BUILD_ID(TASK_SCPPC, conhdl), TASK_APP,
                                                 scppc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Last Scan Interval value to write after discovery
    msg->scan_intv_wd.le_scan_intv = scan_intv;
    ///Last Scan Window value to write after discovery
    msg->scan_intv_wd.le_scan_window = scan_window;
    ///Connection type
    if (scps == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->scps, scps, sizeof(struct scps_content));
    }
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to Write Scan Interval Window Characteristic Value Request. *//**
 * @param[in] scan_intv     Scan Interval value.
 * @param[in] scan_window   Scan Window value.
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  SCPPC_WR_CHAR_RSP or SCPPC_ERROR_IND
 * @description
 *
 *  This API shall be used to inform the Scan Server that the Scan Client has changed its intended
 *  scanning behavior. It will write the Scan Interval Window Characteristic value in the Scan Server database. 
 *
 *  The provided scan parameters are saved within the role task environement so that when the Scan Server requires the 
 *  latest scan parameters (sends a notification for the Scan Refresh Characteristic), these values are automatically 
 *  written in its database.
 *
 ****************************************************************************************
 */
void app_scppc_scan_intv_wd_wr_req(uint16_t scan_intv, uint16_t scan_window, uint16_t conhdl)
{
    struct scppc_scan_intv_wd_wr_req *msg = KE_MSG_ALLOC(SCPPC_SCAN_INTV_WD_WR_REQ, KE_BUILD_ID(TASK_SCPPC, conhdl), TASK_APP,
                                                         scppc_scan_intv_wd_wr_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Scan Interval
    msg->scan_intv_wd.le_scan_intv = scan_intv;
    ///Scan Window
    msg->scan_intv_wd.le_scan_window = scan_window;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to Read Scan Refresh Notification Configuration Value request. *//**
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  SCPPC_SCAN_REFRESH_NTF_CFG_RD_RSP or SCPPC_ERROR_IND
 * @description
 *
 *  This API shall be used to read the value of the Scan Refresh Characteristic Client Characteristic 
 *  Configuration Descriptor.
 *
 ****************************************************************************************
 */
void app_scppc_scan_refresh_ntf_cfg_rd_req(uint16_t conhdl)
{
    struct scppc_scan_refresh_ntf_cfg_rd_req *msg = KE_MSG_ALLOC(SCPPC_SCAN_REFRESH_NTF_CFG_RD_REQ, KE_BUILD_ID(TASK_SCPPC, conhdl), TASK_APP,
                                                                 scppc_scan_refresh_ntf_cfg_rd_req);

    ///Connection handle
    msg->conhdl = conhdl;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to Write Scan Refresh Notification Configuration Value request. *//**
 * @param[in] ntf_cfg       Notification Configuration.
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND 
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  SCPPC_WR_CHAR_RSP or SCPPC_ERROR_IND
 * @description
 *
 *  This API shall be used to either enable or disable notifications for the Scan Refresh Characteristic.
 *
 ****************************************************************************************
 */
void app_scppc_wr_meas_intv_req(uint16_t ntf_cfg, uint16_t conhdl)
{
    struct scppc_scan_refresh_ntf_cfg_req *msg = KE_MSG_ALLOC(SCPPC_SCAN_REFRESH_NTF_CFG_REQ, KE_BUILD_ID(TASK_SCPPC, conhdl), TASK_APP,
                                                scppc_scan_refresh_ntf_cfg_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Notification Configuration
    msg->ntf_cfg = ntf_cfg;

    // Send the message
    ke_msg_send(msg);
}

#endif 
/// @} APP_SCPPC_API
