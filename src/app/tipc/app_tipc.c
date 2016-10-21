/**
 ****************************************************************************************
 *
 * @file app_tipc.c
 *
 * @brief Application TIPC API
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
 * @addtogroup APP_TIPC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_TIP_CLIENT
#include "app_tipc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Time profile Client role - at connection. *//**
 * @param[in] cts           Current Time Service Structure.
 * @param[in] ndcs          Next DST Change Service Structure.
 * @param[in] rtus          Reference Time Update Service Structure.
 * @param[in] conhdl        Connection handle for which the Time Client role is enabled.
 * @response  TIPC_ENABLE_CFM or TIPC_ERROR_IND
 * @description
 *
 *  This API is used for enabling the Client role of the Time profile. This Function 
 *  contains BLE connection handle, the connection type and the previously saved 
 *  discovered CTS, NDCS and RTUS details on peer. 
 *
 *  The connection type may be PRF_CON_DISCOVERY for discovery/initial connection or 
 *  PRF_CON_NORMAL for normal connection. 
 *
 *  For a discovery connection, discovery of the peer CTS, NDCS and RTUS is started and the 
 *  response will be sent at the end of the discovery with the discovered attribute details. 
 *  Application shall save those information to reuse them for other connections. During normal 
 *  connection, previously discovered device information can be reused.
 *
 *  For a normal connection, the response to this request is sent right away after saving the CTS, 
 *  NDCS and RTUS content in the environment and registering TIPC in GATT to receive the 
 *  notifications for the known attribute handle in CTS (Current Time) that would be notified. 
 *  If the Client role is already enabled, a TIPC_ERROR_IND will be sent with the PRF_ERR_REQ_DISALLOWED
 *
 ****************************************************************************************
 */
void app_tipc_enable_req(struct tipc_cts_content *cts,
                         struct tipc_ndcs_content *ndcs,
                         struct tipc_rtus_content *rtus,
                         uint16_t conhdl)
{
    struct tipc_enable_req * msg = KE_MSG_ALLOC(TIPC_ENABLE_REQ, KE_BUILD_ID(TASK_TIPC, conhdl), TASK_APP,
                                                 tipc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (cts == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->cts, cts, sizeof(struct tipc_cts_content));
        memcpy(&msg->ndcs, ndcs, sizeof(struct tipc_ndcs_content));
        memcpy(&msg->rtus, rtus, sizeof(struct tipc_rtus_content));
    }
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a CTS or NDCS or RTUS characteristic value. *//**
 * @param[in] char_code     Code for which characteristic to read.
 *  - TIPC_RD_CTS_CURR_TIME
 *  - TIPC_RD_CTS_LOCAL_TIME_INFO
 *  - TIPC_RD_CTS_REF_TIME_INFO
 *  - TIPC_RD_CTS_CURR_TIME_CLI_CFG
 *  - TIPC_RD_NDCS_TIME_WITH_DST
 *  - TIPC_RD_RTUS_TIME_UPD_STATE
 * @param[in] conhdl        Connection handle for which the Time Client role is enabled.
 * @response  
 *  - TIPC_CT_IND
 *  - TIPC_CT_NTF_CFG_RD_RSP
 *  - TIPC_LTI_RD_RSP
 *  - TIPC_RTI_RD_RSP
 *  - TIPC_TDST_RD_RSP
 *  - TIPC_TUS_RD_RSP
 *
 * @description
 *
 *  This API is used by the application to request sending of a GATT_READ_CHAR_REQ 
 *  with the parameters deduced from the char_code. Upon reception of this message, TIPC checks 
 *  whether the parameters are correct, then if the handle for the characteristic is valid (not 
 *  0x0000), the request is sent to GATT. 
 *
 *  Parsing intelligence of the received response is added in this API handler in order to make 
 *  easier the implementation of an application. When the peer has responded to GATT, and the 
 *  response is routed to TIPC, either the TIPC_CT_IND or the TIPC_CT_NTF_CFG_RD_RSP or the 
 *  TIPC_LTI_RD_RSP or the TIPC_RTI_RD_RSP or the TIPC_TDST_RD_RSP or the  TIPC_TUS_RD_RSP 
 *  message will be generically built following the last read request sent.
 *
 ****************************************************************************************
 */
void app_tipc_rd_char_req(uint8_t char_code, uint16_t conhdl)
{
    struct tipc_rd_char_req *msg = KE_MSG_ALLOC(TIPC_RD_CHAR_REQ, KE_BUILD_ID(TASK_TIPC, conhdl), TASK_APP,
                                                tipc_rd_char_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Characteristic value code
    msg->char_code = char_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the Current Time Characteristic on the Server. *//**
 * @param[in] cfg_val       Stop/notify/indicate value to configure into the peer characteristic:
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] conhdl        Connection handle for which the Time Client role is enabled.
 * @response  TIPC_WR_CHAR_RSP or TIPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_WRITE_CHAR_REQ to the Current 
 *  Time Client Configuration Characteristic Descriptor with the parameter cfg_val. 
 *
 *  When the peer has responded to GATT, and the response is routed to TIPC, the 
 *  TIPC_WR_CHAR_RSP message will be generically built and sent to Application. An error status is 
 *  also possible either for the Write procedure or for the application request, in the second case, 
 *  the TIPC_ERROR_IND message is sent to Application
 *
 ****************************************************************************************
 */
void app_tipc_ct_ntf_cfg_req(uint16_t cfg_val, uint16_t conhdl)
{
    struct tipc_ct_ntf_cfg_req *msg = KE_MSG_ALLOC(TIPC_CT_NTF_CFG_REQ, KE_BUILD_ID(TASK_TIPC, conhdl), TASK_APP,
                                                tipc_ct_ntf_cfg_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    msg->cfg_val = cfg_val;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for writing the Time Update Control Point Characteristic Value on a peer device. *//**
 * @param[in] value         Time Update Control Point value to write:
 *  - TIPS_TIME_UPD_CTNL_PT_GET
 *  - TIPS_TIME_UPD_CTNL_PT_CANCEL
 * @param[in] conhdl        Connection handle for which the Time Client role is enabled.
 * @response  TIPC_WR_CHAR_RSP or TIPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_WRITE_CHAR_REQ to 
 *  the Time Update Control Point Characteristic. Upon reception of this message, TIPC checks 
 *  whether the parameters are correct, then if the handle for the characteristic is valid (not 
 *  0x0000) and whether it is writable or not - if all OK, the request is sent to GATT, otherwise a 
 *  TIPC_ERROR_IND message is build for the Application. 
 *
 *  When the peer has responded to GATT, and the response is routed to TIPC, the 
 *  TIPC_WR_CHAR_RSP message will be generically built and sent to the Application. An error 
 *  status is also possible for the Write procedure, it will be sent through the same message.
 *
 ****************************************************************************************
 */
void app_tipc_wr_time_udp_ctnl_pt_req(uint8_t value, uint16_t conhdl)
{
    struct tipc_wr_time_udp_ctnl_pt_req *msg = KE_MSG_ALLOC(TIPC_WR_TIME_UPD_CTNL_PT_REQ, KE_BUILD_ID(TASK_TIPC, conhdl), TASK_APP,
                                                tipc_wr_time_udp_ctnl_pt_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Value, 1: get reference update, 2: cancel reference update
    ///TIPS_TIME_UPD_CTNL_PT_GET or TIPS_TIME_UPD_CTNL_PT_CANCEL
    msg->value = value;

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_TIPC_API
