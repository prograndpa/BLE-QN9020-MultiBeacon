/**
 ****************************************************************************************
 *
 * @file app_ancsc.c
 *
 * @brief Application ANCSC API
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
 * @addtogroup APP_ANCSC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_ANCS_NC
#include "app_ancsc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Apple Notification Center Service NC role - at connection. *//**
 * @param[in] ancs       Service structure previously discovered in the database of the peer device.
 * @param[in] svc_change Service Change flag, set true to force client to discover the database of 
 * the peer device as of receiving remote Service Change notification.
 * @param[in] conhdl     The connection handle of the connection which the Apple Notification Center Service
 * is enabled.
 * @response  ANCSC_ANCS_CONTENT_IND and ANCSC_CMP_EVT
 * @description
 *
 * This function is used for enabling the NC role of the ANCS. The function contains BLE
 * connection handle, the connection type and the previously saved discovered ANCS details on peer.
 *
 * The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration
 * or PRF_CON_NORMAL (0x01) for a normal connection with a bonded device. Application shall
 * save those information to reuse them for other connections. During normal connection,
 * previously discovered device information can be reused.
 * 
 * For a normal connection, the response to this request is sent right away after saving the
 * ANCS content in the environment and registering ANCS in GATT to receive the notifications
 * for the known attribute handles in ANCS that would be notified.
 *
 * For a discovery connection, discovery of the peer ANCS is started and the response will be
 * sent at the end of the discovery with the discovered attribute details.
 * 
 ****************************************************************************************
 */
void app_ancsc_enable_req(struct ancsc_content *ancs, uint8_t svc_change, uint16_t conhdl)
{
    struct ancsc_enable_cmd *msg = KE_MSG_ALLOC(ANCSC_ENABLE_CMD, KE_BUILD_ID(TASK_ANCSC, conhdl), TASK_APP,
                                                ancsc_enable_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Service Change
    msg->svc_change = svc_change;
    ///Connection type
    if (ancs == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->ancs, ancs, sizeof(struct ancsc_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Subscribe to the characteristic with the property notification. *//**
 * @param[in] disc_code Characteristic code, Possible values are:
 *  - ANCSC_DESC_NTF_SOURCE_CL_CFG
 *  - ANCSC_DESC_DATA_SOURCE_CL_CFG
 * @param[in] cfg_val   Subscribe or stop notification, Possible values are:
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 * @param[in] conhdl    Connection handle for which the ANCS NC role is enabled.
 *
 * @response  ANCSC_CMP_EVT
 * @description
 *
 * This function shall be used by the application to subscribe to the Notification Source 
 * characteristic and Data Source characteristic.
 *
 ****************************************************************************************
 */
void app_ancsc_cfg_indntf_req(uint8_t disc_code, uint16_t cfg_val, uint16_t conhdl)
{
    struct ancsc_cfg_indntf_cmd *cmd = KE_MSG_ALLOC(ANCSC_CFG_INDNTF_CMD, KE_BUILD_ID(TASK_ANCSC, conhdl), TASK_APP,
                                                    ancsc_cfg_indntf_cmd);

    ///Characteristic code
    cmd->disc_code = disc_code;
    ///Configure value
    cmd->cfg_val = cfg_val;
    ///Connection handle
    cmd->conhdl = conhdl;

    // Send the message
    ke_msg_send(cmd);
}

/*
 ****************************************************************************************
 * @brief Get notification attributes command. *//**
 * @param[in] notificationUID A 32-bit numerical value that is the unique identifier (UID) for the iOS notification.
 * @param[in] attribute_size  The length in bytes of the attributes list.
 * @param[in] attribute_list  A pointer to the buffer that contains the attributes list to be transmitted.
 * @param[in] conhdl          Connection handle for which the ANCS NC role is enabled.
 *
 * @response  ANCSC_NTF_SOURCE_IND ANCSC_CMP_EVT
 * @description
 *
 * This function shall be used by the application to retrieve the attributes of a specific iOS 
 * notification or retrieve attributes of a specific app installed on the NP.
 *
 ****************************************************************************************
 */
void app_ancsc_get_ntf_attribute_req(uint32_t notificationUID, uint8_t attribute_size, uint8_t *attribute_list, uint16_t conhdl)
{
    struct ancsc_get_ntf_attribute_cmd *msg = KE_MSG_ALLOC_DYN(ANCSC_GET_NTF_ATTRIBUTE_CMD, KE_BUILD_ID(TASK_ANCSC, conhdl), TASK_APP,
                                                               ancsc_get_ntf_attribute_cmd, attribute_size);

    ///Connection handle
    msg->conhdl = conhdl;
    ///NotificationUID
    msg->notificationUID = notificationUID;
    ///Command size
    msg->attribute_size = attribute_size;
    ///Command
    memcpy(msg->attribute_list, attribute_list, attribute_size);

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Get App attributes command. *//**
 * @param[in] list_size The string identifier of the app, it must be NULL-terminated.
 * @param[in] id_list   A pointer to the buffer that contains the app identifier and attributes
 * attributes to be transmitted.
 * @param[in] conhdl    Connection handle for which the ANCS NC role is enabled.
 *
 * @response  ANCSC_DATA_SOURCE_IND ANCSC_CMP_EVT
 * @description
 *
 * This function shall be used by the application to retrieve the attributes of a specific iOS 
 * notification or retrieve attributes of a specific app installed on the NP.
 *
 ****************************************************************************************
 */
void app_ancsc_get_app_attribute_req(uint16_t list_size, uint8_t *id_list, uint16_t conhdl)
{
    struct ancsc_get_app_attribute_cmd *msg = KE_MSG_ALLOC_DYN(ANCSC_GET_APP_ATTRIBUTE_CMD, KE_BUILD_ID(TASK_ANCSC, conhdl), TASK_APP,
                                                               ancsc_get_app_attribute_cmd, list_size);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Command size
    msg->list_size = list_size;
    ///Command
    memcpy(msg->id_list, id_list, list_size);

    // Send the message
    ke_msg_send(msg);
}

#endif

/// @} BLE_ANCS_NC
