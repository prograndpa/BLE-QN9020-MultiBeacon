/**
 ****************************************************************************************
 *
 * @file app_findl.c
 *
 * @brief Application FMPL API
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
 * @addtogroup APP_FINDL_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_FINDME_LOCATOR
#include "app_findl.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Find Me Locator profile - at connection. *//**
 * @param[in] ias           IAS details.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  FINDL_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Locator role of the Find Me profile. This 
 *  function contains the connection handle for the connection this profile is 
 *  activated, the connection type and the previously saved discovered IAS details on peer.
 *
 *  The connection type may be 0 = Connection for discovery  or 1 = Normal connection. This 
 *  difference has been made and Application would handle it in order to not discover the IAS on 
 *  the Target at every connection, but do it only once and keep the discovered details in the 
 *  Locator device between connections. 
 *  ATTENTION: Normally information about the peer should not be kept from one connection to the next if they have not 
 *  bonded!
 *
 *  If it is a discovery type connection, the IAS parameter is useless, and it will be filled with 0's. 
 *  Otherwise it will contain pertinent data which will be kept in the Locator environment while 
 *  enabled. It allows for the Application to not be aware of attribute details. For a normal 
 *  connection, the response to this request is sent right away after saving the ias content in the 
 *  environment. For a discovery connection, discovery of the peer IAS is started and the response 
 *  will be sent at the end of the discovery with the discovered attribute details.
 *
 ****************************************************************************************
 */
void app_findl_enable_req(struct ias_content *ias, uint16_t conhdl)
{
    struct findl_enable_req * msg = KE_MSG_ALLOC(FINDL_ENABLE_REQ, KE_BUILD_ID(TASK_FINDL, conhdl), TASK_APP,
                                                 findl_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (ias == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->ias, ias, sizeof(struct ias_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Alert level set request. *//**
 * @param[in] alert_lvl     Alert level:
 *  - FINDL_ALERT_NONE
 *  - FINDL_ALERT_MILD
 *  - FINDL_ALERT_HIGH
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  None
 * @description
 *
 *  This API is used by the application to trigger/stop and alert on the peer 
 *  Target device. The Locator role environment contains the attribute handle for the Alert Level 
 *  Characteristic in the IAS of the Target peer device. This way, a correct request to write this 
 *  attribute to the level requested by the application can be sent. Since the Alert Level 
 *  Characteristic in IAS can only be written using a Write No Response ATT Request, no 
 *  confirmation can be received through the profile. The only confirmation can be observed by 
 *  the user either by noticing and alarm on the Target device or the alarm stopping.
 *
 ****************************************************************************************
 */
void app_findl_set_alert_req(uint8_t alert_lvl, uint16_t conhdl)
{
    struct findl_set_alert_req *msg = KE_MSG_ALLOC(FINDL_SET_ALERT_REQ, KE_BUILD_ID(TASK_FINDL, conhdl), TASK_APP,
                                                findl_set_alert_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Alert level
    msg->alert_lvl = alert_lvl;

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_FINDL_API
