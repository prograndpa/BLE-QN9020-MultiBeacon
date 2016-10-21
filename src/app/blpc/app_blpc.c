/**
 ****************************************************************************************
 *
 * @file app_blpc.c
 *
 * @brief Application BLPC API
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
 * @addtogroup APP_BLPC_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_BP_COLLECTOR
#include "app_blpc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Blood Pressure Collector profile - at connection. *//**
 * @param[in] bps           Blood Pressure Service Content Structure.
 * @param[in] conhdl        Connection handle for which the profile blood pressure collector role is enabled.
 * @response  BLPC_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Collector role of the Blood Pressure profile. This Function 
 *  contains BLE connection handle, the connection type and the previously saved discovered BPS and DIS 
 *  details on peer. 
 *
 *  The connection type may be 0 = Connection for discovery/initial configuration or 1 = Normal connection. This 
 *  parameter is used by Application to discover peer device services once at first connection. Application shall save those 
 *  information to reuse them for other connections. During normal connection, previously discovered device 
 *  information can be reused.
 *
 *  This is usefull since most use cases allow Bood Pressure Sensor to disconnect the link once all measurements have 
 *  been sent to Collector.
 *
 *  If it is a discovery /configuration type of connection, the BPS and DIS parameters are useless, they will be filled with 
 *  0's. 
 *
 *  Otherwise they will contain pertinent data which will be kept in the Collector environment while enabled. It allows for 
 *  the Application to not be aware of attribute details. 
 *
 *  For a normal connection, the response to this request is sent right away after saving the BPS and DIS content in the 
 *  environment and registering BLPC in GATT to receive the indications and notifications for the known attribute handles
 *  in BPS that would be notified/indicated. For a discovery connection, discovery of the peer BPS and DIS is started and 
 *  the response will be sent at the end of the discovery with the discovered attribute details. 
 *
 ****************************************************************************************
 */
void app_blpc_enable_req(struct bps_content *bps, uint16_t conhdl)
{
    struct blpc_enable_req * msg = KE_MSG_ALLOC(BLPC_ENABLE_REQ, KE_BUILD_ID(TASK_BLPC, conhdl), TASK_APP,
                                                 blpc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (bps == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->bps, bps, sizeof(struct bps_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a BPS characteristic value. *//**
 * @param[in] char_code     Code for which characteristic to read:
 *  - BLPC_RD_BPS_FEATURE     ///Read BPS Blood pressure Features 
 *  - BLPC_RD_BPS_BP_MEAS_CFG ///Read BPS Blood pressure Measurement Client Cfg. Desc
 *  - BLPC_RD_BPS_CP_MEAS_CFG ///Read BPS Intermdiate Cuff Pressure Client Cfg. Desc
 * @param[in] conhdl        Connection handle for which the profile blood pressure collector role is enabled.
 * @response  BLPC_RD_CHAR_RSP or BLPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_READ_CHAR_REQ with the parameters 
 *  deduced from the char_code. The definitions for the different mapping codes for characteristics that are possibly 
 *  readable are in blpc.h (for BPS) and in svc.h (for DIS). Upon reception of this message, BLPC checks whether the 
 *  parameters are correct, then if the handle for the characteristic is valid (not 0x0000) and the request is sent to GATT. 
 *  When the peer has responded to GATT, and the response is routed to BLPC, the BLPC_RD_CHAR_RSP message will be 
 *  generically built and the Application must be able to interpret it based on the read request it made. And error status 
 *  is also possible either for the Read procedure or for the application request, in the second case, the BLPC_ERROR_IND 
 *  message is sent to Application.
 *
 ****************************************************************************************
 */
void app_blpc_rd_char_req(uint8_t char_code, uint16_t conhdl)
{
    struct blpc_rd_char_req *msg = KE_MSG_ALLOC(BLPC_RD_CHAR_REQ, KE_BUILD_ID(TASK_BLPC, conhdl), TASK_APP,
                                                blpc_rd_char_req);
    ///Characteristic value code
    msg->char_code = char_code;
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 2 characteristics that can be handled. *//**
 * @param[in] char_code     Code for which characteristic to read:
 *  - BPS_BP_MEAS_CODE    ///Blood Pressure Measurement
 *  - BPS_INTERM_CP_CODE  ///Intermediate Cuff Pressure Measurement
 * @param[in] cfg_val       Configuration characteristics:
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] conhdl        Connection handle for which the profile blood pressure collector role is enabled.
 * @response  BLPC_WR_CHAR_RSP or BLPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_WRITE_CHAR_REQ with the parameters 
 *  deduced from the char_code and cfg_val. The definitions for the different codes for characteristics that can be 
 *  configured to indicate/notify are in blpc.h. Upon reception of this message, BLPC checks whether the parameters are 
 *  correct, then if the handle for the characteristic is valid (not 0x0000) and the request is sent to GATT. When the peer 
 *  has responded to GATT, and the response is routed to BLPC, the BLPC_WR_CHAR_RSP message will be generically 
 *  built and sent to Application. An error status is also possible either for the Write procedure or for the application 
 *  request, in the second case, the BLPC_ERROR_IND message is sent to Application.
 *
 ****************************************************************************************
 */
void app_blpc_cfg_indntf_req(uint8_t char_code, uint16_t cfg_val, uint16_t conhdl)
{
    struct blpc_cfg_indntf_req *msg = KE_MSG_ALLOC(BLPC_CFG_INDNTF_REQ, KE_BUILD_ID(TASK_BLPC, conhdl), TASK_APP,
                                                blpc_cfg_indntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Own code for differentiating between blood pressure and intermediate cuff pressure measurements
    msg->char_code = char_code;
    ///Stop/notify/indicate value to configure into the peer characteristic
    msg->cfg_val = cfg_val;
    
    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_BLPC_API
