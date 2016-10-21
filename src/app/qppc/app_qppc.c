/**
 ****************************************************************************************
 *
 * @file app_qppc.c
 *
 * @brief Application QPPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_QPPC_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_QPP_CLIENT
#include "app_qppc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the profile - at connection.    *//**
 * @param[in] qpps          Quintic Private Service Content Structure
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  QPPC_ENABLE_CFM
 * @description
 *  This API is used for enabling the Client role of the QPP. Profile will discovery all
 *  the attributes related to QPPS. This Function contains BLE connection handle, the connection type 
 *  and the previously saved and discovered QPPS details on peer. The connection type may be 
 *  PRF_CON_DISCOVERY (0x00) for discovery/initial configuration or PRF_CON_NORMAL (0x01) for a normal 
 *  connection with a bonded device. Application shall save those information to reuse them for 
 *  other connections. During normal connection, previously discovered device information can be reused.
 *
 *  If it is a discovery/configuration type of connection, it is useless to fill the qpps parameters.  
 *  Otherwise they will contain pertinent data which will be kept in the Client environment while enabled. 
 *
 *  For a normal connection, the response to this request is sent right away after saving the qpps content in the 
 *  environment and registering QPPC in GATT to receive the notifications for the known attribute handles in QPPS that 
 *  would be notified. For a discovery connection, discovery of the peer QPPS is started and 
 *  the response will be sent at the end of the discovery with the discovered attribute details. 
 *
 ****************************************************************************************
 */
void app_qppc_enable_req(struct qpps_content *qpps, uint16_t conhdl)
{
    struct qppc_enable_req * msg = KE_MSG_ALLOC(QPPC_ENABLE_REQ, KE_BUILD_ID(TASK_QPPC, conhdl), TASK_APP,
                                                qppc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (qpps == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->qpps, qpps, sizeof(struct qpps_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a QPPS characteristic descriptor values. *//**
 * @param[in] conhdl        Connection handle for which the profile Heart Rate Collector role is enabled
 * @param[in] char_code     Code for which characteristic to read:
 *  - QPPC_QPPS_RX_CHAR_VALUE_USER_DESP
 *  - QPPC_QPPS_FIRST_TX_VALUE_CLI_CFG
 * @response  QPPC_RD_CHAR_RSP or QPPC_ERROR_IND
 * @description
 * This API shall be used to read the value of a descriptor in the peer device database.
 *
 * @note
 * The number of TX characteristic can be configured by server, so the maximum value of char_code parameter
 * depends on nb_ntf_char variable in structure qppc_enable_cfm, and it should be nb_ntf_char.
 * 
 ****************************************************************************************
 */
void app_qppc_rd_char_req(uint16_t conhdl, uint8_t char_code)
{
    struct qppc_rd_char_req *msg = KE_MSG_ALLOC(QPPC_RD_CHAR_REQ, KE_BUILD_ID(TASK_QPPC, conhdl), TASK_APP,
                                                qppc_rd_char_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///Characteristic value code
    msg->char_code = char_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the characteristics that can be handled. *//**
 * @param[in] cfg_val       Stop/notify value to configure into the peer characteristic.
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 * @param[in] conhdl        Connection handle
 * @param[in] char_code     Code for which characteristic to read:
 *  - QPPC_QPPS_FIRST_TX_VALUE_CLI_CFG
 * @response  QPPC_WR_CHAR_RSP or QPPC_ERROR_IND
 * @description
 *  This API shall be used to enable or disable the notifications for one of the Characteristic.
 *
 * @note
 * The number of TX characteristic can be configured by server, so the maximum value of char_code parameter
 * depends on nb_ntf_char variable in structure qppc_enable_cfm, and it should be nb_ntf_char.
 * 
 ****************************************************************************************
 */
void app_qppc_cfg_indntf_req(uint16_t cfg_val, uint16_t conhdl, uint8_t char_code)
{
    struct qppc_cfg_indntf_req *msg = KE_MSG_ALLOC(QPPC_CFG_INDNTF_REQ, KE_BUILD_ID(TASK_QPPC, conhdl), TASK_APP,
                                                   qppc_cfg_indntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Stop/notify value to configure into the peer characteristic
    msg->cfg_val = cfg_val;
    ///Char code
    msg->char_code = char_code;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send data to server.    *//**
 * @param[in] len           Length of data to be sent
 * @param[in] val           Pointer to data to be sent
 * @param[in] conhdl        Connection handle
 * @response  QPPC_ERROR_IND
 * @description
 * This function is used by the application to send a raw data to server.
 * 
 ****************************************************************************************
 */
void app_qppc_wr_data_req(uint8_t len, uint8_t *val, uint16_t conhdl)
{
    struct qppc_wr_data_req *msg = KE_MSG_ALLOC_DYN(QPPC_WR_DATA_REQ, KE_BUILD_ID(TASK_QPPC, conhdl), TASK_APP,
                                                qppc_wr_data_req, len);

    ///Connection handle
    msg->conhdl = conhdl;
    msg->length = len;
    memcpy(msg->data, val, msg->length);

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_QPPC_API
