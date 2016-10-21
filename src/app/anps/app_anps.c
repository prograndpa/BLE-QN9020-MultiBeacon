/**
 ****************************************************************************************
 *
 * @file app_anps.c
 *
 * @brief Application ANPS API
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
 * @addtogroup APP_ANPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_AN_SERVER
#include "app_anps.h"

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
 * @brief Create the Alert Notification Profile server database - at initiation  *//**
 * 
 * @param[in] supp_new_alert_cat      New Alert Category Characteristic
 * @param[in] supp_unread_alert_cat   Unread Alert Category Characteristic
 * @response  ANPS_CMP_EVT
 * @description
 *
 * This API message shall be used to add one instance of the Alert Notification Service 
 * in the database.
 * 
 ****************************************************************************************
 */
void app_anps_create_db(uint16_t supp_new_alert_cat, uint16_t supp_unread_alert_cat)
{
    struct anps_create_db_req *msg = KE_MSG_ALLOC(ANPS_CREATE_DB_REQ, TASK_ANPS, TASK_APP,
                                                  anps_create_db_req);
    ///Supported New Alert Category Characteristic Value
    msg->supp_new_alert_cat.cat_id_mask_0 = (uint8_t)(supp_new_alert_cat & 0x00ff);
    msg->supp_new_alert_cat.cat_id_mask_1 = (uint8_t)((supp_new_alert_cat >> 8) & 0x00ff);
    ///Supported Unread Alert Category Characteristic Value
    msg->supp_unread_alert_cat.cat_id_mask_0 = (uint8_t)(supp_unread_alert_cat & 0x00ff);
    msg->supp_unread_alert_cat.cat_id_mask_1 = (uint8_t)((supp_unread_alert_cat >> 8) & 0x00ff);

    // Send the message
    ke_msg_send(msg);
}

 /*
 ****************************************************************************************
 * @brief Start the Alert Notification Profile server role - at connection        *//**
 * 
 * @param[in] conhdl    Connection handle for which the Alert Notification Profile Server 
 *                      role is enabled.
 * @param[in] sec_lvl             Required security level.
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type            Connection Type:configuration(0) or discovery(1)
 * @param[in] new_alert_ntf_cfg   New  Alert Characteristic - Saved Client Characteristic 
 *                                Configuration Descriptor Value for a bonded device.
 * @param[in] unread_alert_status_ntf_cfg   Unread Alert Status Characteristic - Saved Client
 *                                          Characteristic Configuration Descriptor Value for 
 *                                          a bonded device.
 * @response  ANPS_CMP_EVT
 * @description
 *
 * This API message shall be used after the connection with a peer device has been established
 * in order to enable the ANP Server role task for the specified connection. 
 * 
 * Application shall provide connection handle in order to activate the profile. Connection 
 * handle and Application task ID are saved within the role's environment.
 * 
 * Connection Type will determine if notification configurations should be applied to the 
 * corresponding ANS Attributes in the database:
 *     - Normal connection: Peer device is known (bonded) and client configuration characteristics
 *       values shall  be restored. 
 *     - Discovery connection: Peer device is unknown and peer collector will manage client
 *       configuration characteristics.
 * The security level is very important because it allows the application to modulate the 
 * protection of the attributes related to the profile. The implementation only allows modulation
 * of the read/write/notify permissions (!not properties) of the characteristic values. (if 
 * unauthenticated is requested, then a Read/Write to a characteristic won't be allowed if the 
 * link between the devices is not unauthenticated level of security. 
 ****************************************************************************************
 */
void app_anps_enable_req( uint8_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t new_alert_ntf_cfg,
                         uint16_t unread_alert_status_ntf_cfg)
{
    struct anps_enable_req *msg = KE_MSG_ALLOC(ANPS_ENABLE_REQ, KE_BUILD_ID(TASK_ANPS, conhdl), TASK_APP,
                                                anps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->new_alert_ntf_cfg = new_alert_ntf_cfg;
    msg->unread_alert_status_ntf_cfg = unread_alert_status_ntf_cfg;

    // Send the message
    ke_msg_send(msg);
}

 /*
 ****************************************************************************************
 * @brief Update the value of a characteristic - at connection or idle  *//**
 * 
 * @param[in] conhdl    Connection handle for which the Alert Notification Profile Server 
 *                      role is enabled.
 * @param[in] operation
 *                      - ANPS_UPD_NEW_ALERT_OP_CODE
 *                      - ANPS_UPD_UNREAD_ALERT_STATUS_OP_CODE 
 * @param[in] value     New Alert or Unread Alert Status.
 * @description
 *
 * This API message shall be used by the application to send a New Alert or an Unread Alert
 * Status notification to a specific device.
 * 
 * The implementation of the profile takes care of the provided value (category id, бн), if
 * one of these is not within the ranges defined by the profile specification, an ANPS_CMP_EVT
 * message with an PRF_ERR_INVALID_PARAM status is sent to the application.
 * 
 * It also checks whether the provided category has been set as supported and if the peer
 * device has enable sending of notifications for this category and for the characteristic
 * to update. If no notification can be sent, the received status will be PRF_ERR_NTF_DISABLED.
 * 
 * In the case where a New Alert value is sent, this message shall be allocated dynamically
 * using the KE_ALLOC_MSG_DYN macro because the length of the information string can be
 * different between messages.
 ****************************************************************************************
 */
void app_anps_ntf_alert_cmd(uint8_t conhdl, uint8_t operation, union anps_value_tag *value)
{
    struct anps_ntf_alert_cmd *msg;

    if (operation == ANPS_UPD_NEW_ALERT_OP_CODE)
    {
        msg = KE_MSG_ALLOC_DYN(ANPS_NTF_ALERT_CMD, KE_BUILD_ID(TASK_ANPS, conhdl), TASK_APP, anps_ntf_alert_cmd,
                               value->new_alert.info_str_len);
    }
    else
    {
        msg = KE_MSG_ALLOC(ANPS_NTF_ALERT_CMD, KE_BUILD_ID(TASK_ANPS, conhdl), TASK_APP, anps_ntf_alert_cmd);
    }

    msg->conhdl = conhdl;
    msg->operation = operation;
    msg->value = *value;

    if (operation == ANPS_UPD_NEW_ALERT_OP_CODE)
    {
        memcpy(msg->value.new_alert.str_info, value->new_alert.str_info, value->new_alert.info_str_len);
    }
    
    // Send the message
    ke_msg_send(msg);
}
#endif // BLE_AN_SERVER

/// @} APP_ANPS_API
