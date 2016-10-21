/**
 ****************************************************************************************
 *
 * @file app_basc.c
 *
 * @brief Application BASC API
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
 * @addtogroup APP_BASC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_BATT_CLIENT
#include "app_basc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Battery Service Client Role - at connection.  *//**
 * @param[in] bas_nb        Number of BAS instances that have previously been found.
 * @param[in] bas           Battery Service Content Structure.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  BASC_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Client role of the BAS. This Function contains BLE 
 *  connection handle, the connection type and the previously saved and discovered BAS details on peer. 
 *  The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration or PRF_CON_NORMAL 
 *  (0x01) for a normal connection with a bonded device.  Application shall save those information to reuse them for 
 *  other connections. During normal connection, previously discovered device information can be reused. 
 * 
 *  If it is a discovery/configuration type of connection, it is useless to fill the BAS parameters (bas_nb and bas).  
 *  Otherwise they will contain pertinent data which will be kept in the Client environment while enabled. 
 *
 *  For a normal connection, the response to this request is sent right away after saving the BAS content in the 
 *  environment and registering BASC in GATT to receive the notifications for the known attribute handles in BAS that 
 *  would be notified (Battery Level Characteristic). For a discovery connection, discovery of the peer BAS is started and 
 *  the response will be sent at the end of the discovery with the discovered attribute details. 
 *
 ****************************************************************************************
 */
void app_basc_enable_req(uint8_t bas_nb, struct bas_content *bas, uint16_t conhdl)
{
    struct basc_enable_req * msg = KE_MSG_ALLOC(BASC_ENABLE_REQ, KE_BUILD_ID(TASK_BASC, conhdl), TASK_APP,
                                                 basc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Number of BAS instances that have previously been found
    msg->bas_nb = bas_nb;
    ///Connection type
    if (bas == NULL || bas_nb == 0)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->bas[0], bas, bas_nb*sizeof(struct bas_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Read Characteristic Value Request. *//**
 * @param[in] char_code     Battery Service Characteristic Code:
 *  - BASC_RD_BAS_BATT_LEVE
 *  - BASC_RD_BAS_BATT_LEVEL_PRES_FORMAT
 *  - BASC_RD_BAS_BATT_LEVEL_CLI_CFG
 * @param[in] bas_nb        Number of BAS instances that have previously been found.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  
 *  - BASC_BATT_LEVEL_NTF_CFG_RD_RSP 
 *  - BASC_BATT_LEVEL_PRES_FORMAT_RD_RSP 
 *  - BASC_ERROR_IND
 *
 * @description
 *
 *  This API shall be used to read the value of a characteristic or a descriptor in the peer device 
 *  database. 
 *
 ****************************************************************************************
 */
void app_basc_rd_char_req(uint8_t char_code, uint8_t bas_nb, uint16_t conhdl)
{
    struct basc_char_rd_req *msg = KE_MSG_ALLOC(BASC_CHAR_RD_REQ, KE_BUILD_ID(TASK_BASC, conhdl), TASK_APP,
                                                basc_char_rd_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Characteristic Code
    msg->char_code = char_code;
    ///Battery Service Instance - From 0 to BASC_NB_BAS_INSTANCES_MAX-1
    msg->bas_nb = bas_nb;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Write Battery Level Notification Configuration Value request. *//**
 * @param[in] ntf_cfg       BAS Client configuration characteristics:
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] bas_nb        Number of BAS instances that have previously been found.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  BASC_WR_CHAR_RSP or BASC_ERROR_IND
 * @description
 *
 *  This API shall be used to enable or disable the notifications for one of the Battery Level 
 *  Characteristic.
 *
 ****************************************************************************************
 */
void app_basc_cfg_indntf_req(uint16_t ntf_cfg, uint8_t bas_nb, uint16_t conhdl)
{
    struct basc_batt_level_ntf_cfg_req *msg = KE_MSG_ALLOC(BASC_BATT_LEVEL_NTF_CFG_REQ, KE_BUILD_ID(TASK_BASC, conhdl), TASK_APP,
                                                basc_batt_level_ntf_cfg_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Notification Configuration
    msg->ntf_cfg = ntf_cfg;
    ///Battery Service Instance - From 0 to BASC_NB_BAS_INSTANCES_MAX-1
    msg->bas_nb = bas_nb;
    
    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_BASC_API
