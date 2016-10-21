/**
 ****************************************************************************************
 *
 * @file app_htpc.c
 *
 * @brief Application HTPC API
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
 * @addtogroup APP_HTPC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HT_COLLECTOR
#include "app_htpc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Health Thormometer Collector profile - at connection. *//**
 * @param[in] hts           HTS details.
 * @param[in] conhdl        Connection handle for which the profile Collector role is enabled.
 * @response  HTPC_ENABLE_CFM
 * @description  
 *
 * This API is used by Application to send message to TASK_HTPC for enabling the Collector role 
 * of the Health Thermometer profile, and it contains the connection handle for the connection this 
 * profile is activated, the connection type and the previously saved discovered HTS details on peer. 
 * 
 * The connection type may be 0 = Connection for discovery/initial configuration or 1 = Normal 
 * connection. This difference has been made and Application would handle it in order to not 
 * discover the HTS on the Thermometer at every connection, but do it only once and keep the 
 * discovered details in the Collector device between connections.  Configuration can be done 
 * during a normal connection also, but since most use cases allow Thermometer to disconnect 
 * the link once all measurements have been sent to Collector, the Collector may not have the 
 * time for it.
 *  
 * If it is a discovery /configuration type of connection, the hts and dis parameters are useless, 
 * they will be filled with 0's. Otherwise they will contain pertinent data which will be kept in the 
 * Collector environment while enabled. It allows for the Application to not be aware of attribute 
 * details. 
 *    
 * For a normal connection, the response to this request is sent right away after saving the HTS 
 * and DIS content in the environment and registering HTPC in GATT to receive the indications 
 * and notifications for the known attribute handles in HTS that would be notified/indicated. For a 
 * discovery connection, discovery of the peer HTS is started and the response will be sent at the 
 * end of the discovery with the discovered attribute details.
 *    
 ****************************************************************************************
 */
void app_htpc_enable_req(struct htpc_hts_content *hts, uint16_t conhdl)
{
    struct htpc_enable_req * msg = KE_MSG_ALLOC(HTPC_ENABLE_REQ, KE_BUILD_ID(TASK_HTPC, conhdl), TASK_APP,
                                                 htpc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (hts == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->hts, hts, sizeof(struct htpc_hts_content));
    }
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a HTs or DIS characteristic value. *//**
 * @param[in] char_code     Health Thermometer Service Characteristics
 *  - HTPC_RD_HTS_TEMP_TYPE
 *  - HTPC_RD_HTS_MEAS_INTV
 *  - HTPC_RD_HTS_TEMP_MEAS_CLI_CFG
 *  - HTPC_RD_HTS_INTM_TEMP_CLI_CFG
 *  - HTPC_RD_HTS_MEAS_INTV_CLI_CFG
 *  - HTPC_RD_HTS_MEAS_INTV_VAL_RGE
 * @param[in] conhdl        Connection handle for which the profile Collector role is enabled.
 * @response  HTPC_RD_CHAR_RSP or HTPC_ERROR_IND
 * @description 
 *
 * This API is used by Application to send a GATT_READ_CHAR_REQ with the parameters deduced from 
 * the char_code. Upon reception of this message, HTPC checks whether the parameters are correct, 
 * then if the handle for the characteristic is valid (not 0x0000) and the request is sent to GATT. 
 * When the peer has responded to GATT, and the response is routed to HTPC, the HTPC_RD_CHAR_RSP 
 * message will be generically built and the Application must be able to interpret it based on the 
 * read request it made. And error status is also possible either for the Read procedure or for the 
 * application request, in the second case, the HTPC_ERROR_IND message is sent to Application. No parsing 
 * intelligence of the received response is added in this API handler, so all the work of interpretation 
 * must be added in the Application depending of its request and use of the response.
 *
 ****************************************************************************************
 */
void app_htpc_rd_char_req(uint8_t char_code, uint16_t conhdl)
{
    struct htpc_rd_char_req *msg = KE_MSG_ALLOC(HTPC_RD_CHAR_REQ, KE_BUILD_ID(TASK_HTPC, conhdl), TASK_APP,
                                                htpc_rd_char_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///Characteristic value code
    msg->char_code = char_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 3 characteristics that can be handled. *//**
 * @param[in] char_code     Health Thermometer Service Characteristics
 *  - HTPC_CHAR_HTS_TEMP_MEAS
 *  - HTPC_CHAR_HTS_INTM_TEMP
 *  - HTPC_CHAR_HTS_MEAS_INTV
 * @param[in] cfg_val       Possible values for setting client configuration characteristics
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] conhdl        Connection handle for which the profile Collector role is enabled.
 * @response  HTPC_WR_CHAR_RSP or HTPC_ERROR_IND
 * @description 
 *
 * This API is used by the application to send a GATT_WRITE_CHAR_REQ with the parameters deduced 
 * from the char_code and cfg_val. The definitions for the different codes for characteristics that 
 * can be configured to indicate/notify are in htpc.h. Upon reception of this message, HTPC checks 
 * whether the parameters are correct, then if the handle for the characteristic is valid (not 0x0000) 
 * and the request is sent to GATT. When the peer has responded to GATT, and the response is routed 
 * to HTPC, the HTPC_WR_CHAR_RSP message will be generically built and sent to Application. An error 
 * status is also possible either for the Write procedure or for the application request, in the second 
 * case, the HTPC_ERROR_IND message is sent to Application. 
 *
 ****************************************************************************************
 */
void app_htpc_cfg_indntf_req(uint8_t char_code, uint16_t cfg_val, uint16_t conhdl)
{
    struct htpc_cfg_indntf_req *msg = KE_MSG_ALLOC(HTPC_CFG_INDNTF_REQ, KE_BUILD_ID(TASK_HTPC, conhdl), TASK_APP,
                                                htpc_cfg_indntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    msg->cfg_val = cfg_val;
    ///Own code for differentiating between Temperature Measurement, Intermediate Temperature and Measurement Interval chars
    msg->char_code = char_code;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief APP request for measurement interval write. *//**
 * @param[in] intv          range should between 1s ~ 65535s
 * @param[in] conhdl        Connection handle for which the profile Collector role is enabled.
 * @response  HTPC_WR_CHAR_RSP or HTPC_ERROR_IND
 * @description 
 *
 * This API is used by the application to send a GATT_WRITE_CHAR_REQ to 
 * the HTS Measurement Interval Char. 
 * Upon reception of this message, HTPC checks whether the parameters are correct, then if the 
 * handle for the characteristic is valid (not 0x0000) and whether it is writable or not - if all OK, 
 * the request is sent to GATT, otherwise a HTPC_ERROR_IND message is built for the Application. 
 * When the peer has responded to GATT, and the response is routed to HTPC, the 
 * HTPC_WR_CHAR_RSP message will be generically built and sent to Application. An error status 
 * is also possible for the Write procedure, it will be sent through that same message. It is the 
 * application's responsibility   to write a measurement interval value that respects the valid 
 * range in HTS characteristics.
 *
 ****************************************************************************************
 */
void app_htpc_wr_meas_intv_req(uint16_t intv, uint16_t conhdl)
{
    struct htpc_wr_meas_intv_req *msg = KE_MSG_ALLOC(HTPC_WR_MEAS_INTV_REQ, KE_BUILD_ID(TASK_HTPC, conhdl), TASK_APP,
                                                htpc_wr_meas_intv_req);

    ///Connection handle
    msg->conhdl = conhdl;
    // range should between 1s ~ 65535s
    msg->intv = intv;

    // Send the message
    ke_msg_send(msg);
}

#endif 
/// @} APP_HTPC_API
