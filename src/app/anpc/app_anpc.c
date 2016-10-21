/**
 ****************************************************************************************
 *
 * @file app_anpc.c
 *
 * @brief Application ANPC API
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
 * @addtogroup APP_ANPC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_AN_CLIENT
#include "app_anpc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
 /*
 ****************************************************************************************
 * @brief Start the Alert Notification Status profile Client role - at connection. *//**
 * @param[in] new_alert_enable       New Alert Category to Enable for Notifications.
 * @param[in] unread_alert_enable    Unread Alert Category to Enable for Notifications.
 * @param[in] ans                    Service structure previously discovered in the
 *                                   database of the peer device.
 * @param[in] conhdl                 Connection handle for which the Alert Notification
 *                                   Status Client role is enabled.
 * @response  ANPC_ANS_CONTENT_IND, ANPC_VALUE_IND and PASPC_CMP_EVT
 * @description
 *
 * This API message is used for enabling the Client role of the ANP. This Application 
 * message contains BLE connection handle, the connection type and the previously saved 
 * discovered ANS details on peer.
 * 
 * The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration
 * or PRF_CON_NORMAL (0x01) for a normal connection with a bonded device. Application shall
 * save those information to reuse them for other connections. During normal connection,
 * previously discovered device information can be reused.
 * 
 * For a normal connection, the response to this request is sent right away after saving the
 * ANS content in the environment and registering ANPC in GATT to receive the notifications
 * for the known attribute handles in ANS that would be notified.
 * 
 * For a discovery connection, discovery of the peer ANS is started and the response will be
 * sent at the end of the discovery with the discovered attribute details.
 * 
 ****************************************************************************************
 */
void app_anpc_enable_req(struct anp_cat_id_bit_mask *new_alert_enable,
                         struct anp_cat_id_bit_mask *unread_alert_enable,
                         struct anpc_ans_content *ans,
                         uint16_t conhdl)
{
    struct anpc_enable_cmd *msg = KE_MSG_ALLOC(ANPC_ENABLE_CMD, KE_BUILD_ID(TASK_ANPC, conhdl), TASK_APP,
                                                 anpc_enable_cmd);

    ///Connection handle
    msg->conhdl = conhdl;

    ///Connection type
    if (ans == NULL)
    {
        //msg->new_alert_enable = *new_alert_enable;
        //msg->unread_alert_enable = *unread_alert_enable;
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
       // msg->new_alert_enable = *new_alert_enable;
        //msg->unread_alert_enable = *unread_alert_enable;

        memcpy(&msg->ans, ans, sizeof(struct anpc_ans_content));
    }

    // Send the message
    ke_msg_send(msg);
}

 /*
 ****************************************************************************************
 * @brief Generic message to read a New Alert CFG or Unread Alert Status CFG characteristic
 *        value. *//**
 * @param[in] read_code
 *  - ANPC_RD_WR_NEW_ALERT_CFG
 *  - ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG
 * @param[in] conhdl          Connection handle for which the Alert Notification Status
 *                            Client role is enabled.
 * @response  ANPC_VALUE_IND and ANPC_CMP_EVT
 * @description
 *
 * The API message shall be used to read the value of an attribute in the peer device database.
 * 
 ****************************************************************************************
 */
void app_anpc_rd_char_req(uint8_t read_code, uint16_t conhdl)
{
    struct anpc_read_cmd *msg = KE_MSG_ALLOC(ANPC_READ_CMD, KE_BUILD_ID(TASK_ANPC, conhdl), TASK_APP,
                                                anpc_read_cmd);
    ///read code
    msg->read_code = read_code;
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

 /*
 ****************************************************************************************
 * @brief Generic message to write characteristic. *//**
 * @param[in] write_code
 *  - ANPC_WR_ALERT_NTF_CTNL_PT
 *  - ANPC_RD_WR_NEW_ALERT_CFG
 *  - ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG
 * @param[in] value
 *  - Alert Notification Control Point
 *  - New Alert Notification Configuration
 *  - Unread Alert Status Notification Configuration
 * @param[in] conhdl          Connection handle for which the Alert Notification Status
 *                            Client role is enabled.
 * @response  ANPC_CMP_EVT
 * @description
 *
 * This API message shall be used by the application to write the value of one of the writable
 * attribute in the peer device database.
 * 
 ****************************************************************************************
 */
void app_anpc_wr_char_req(uint8_t write_code, union anpc_write_value_tag *value, uint16_t conhdl)
{
    struct anpc_write_cmd *msg = KE_MSG_ALLOC(ANPC_WRITE_CMD, KE_BUILD_ID(TASK_ANPC, conhdl), TASK_APP,
                                                anpc_write_cmd);
    ///Write code
    msg->write_code = write_code;
    ///Value
    msg->value = *value;
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}
#endif

/// @} APP_ANPC_API
