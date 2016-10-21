/**
 ****************************************************************************************
 *
 * @file app_disc.c
 *
 * @brief Application DISC API
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
 * @addtogroup APP_DISC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_DIS_CLIENT
#include "app_disc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Device Information profile - at connection. *//**
 * @param[in] dis           Device Information Service Content Structure.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  DISC_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Client role of the Device Information Service. 
 *  The Application sends it, and it contains the connection handle for the connection this 
 *  profile is activated, the connection type and the previously saved discovered DIS details on 
 *  peer.  
 *
 *  The connection type may be 0 = Connection for discovery  or 1 = Normal connection. This 
 *  difference has been made and Application would handle it in order to not discover the DIS on 
 *  the Server at every connection, but do it only once and keep the discovered details in the 
 *  Client device between connections. 
 *
 *  If it is a discovery type connection, the DIS parameter is useless, and it will be filled with 0's. 
 *  Otherwise it will contain pertinent data which will be kept in the Client environment while 
 *  enabled. It allows for the Application to not be aware of attribute details. For a normal 
 *  connection, the response to this request is sent right away after saving the dis content in the 
 *  environment. For a discovery connection, discovery of the peer DIS is started and the response 
 *  will be sent at the end of the discovery with the discovered attribute details.
 *
 ****************************************************************************************
 */
void app_disc_enable_req(struct disc_dis_content *dis, uint16_t conhdl)
{
    struct disc_enable_req * msg = KE_MSG_ALLOC(DISC_ENABLE_REQ, KE_BUILD_ID(TASK_DISC, conhdl), TASK_APP,
                                                 disc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (dis == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->dis, dis, sizeof(struct disc_dis_content));
    }
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a DIS characteristic value. *//**
 * @param[in] char_code     Code of the characteristic:
 *  - DISC_MANUFACTURER_NAME_CHAR
 *  - DISC_MODEL_NB_STR_CHAR
 *  - DISC_SERIAL_NB_STR_CHAR
 *  - DISC_HARD_REV_STR_CHAR
 *  - DISC_FIRM_REV_STR_CHAR
 *  - DISC_SW_REV_STR_CHAR
 *  - DISC_SYSTEM_ID_CHAR
 *  - DISC_IEEE_CHAR
 *  - DISC_PNP_ID_CHAR
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  DISC_RD_CHAR_RSP or DISC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_READ_CHAR_REQ 
 *  with the parameters deduced from the  char_code. Upon reception of this message, DISC 
 *  checks whether the parameters are correct, then if the handle for the characteristic is valid 
 *  (not 0x0000) and the request is sent to GATT.
 *
 *  When the peer has responded to GATT, and the response is routed to DISC, the 
 *  DICS_RD_CHAR_RSP message will be generically built and the Application must be able to 
 *  interpret it based on the read request it made. And error status is also possible either for the 
 *  Read procedure or for the application request, in the second case, the DISC_ERROR_IND 
 *  message is sent to Application.
 *
 *  No parsing intelligence of the received response is added in this API handler, so all the work of 
 *  interpretation must be added in the Application depending of its request and use of the 
 *  response.
 *
 ****************************************************************************************
 */
void app_disc_rd_char_req(uint8_t char_code, uint16_t conhdl)
{
    struct disc_rd_char_req *msg = KE_MSG_ALLOC(DISC_RD_CHAR_REQ, KE_BUILD_ID(TASK_DISC, conhdl), TASK_APP,
                                                disc_rd_char_req);
    ///Characteristic value code
    msg->char_code = char_code;
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_DISC_API
