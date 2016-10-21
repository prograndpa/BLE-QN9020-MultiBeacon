/**
 ****************************************************************************************
 *
 * @file app_anps_task.c
 *
 * @brief Application ANPS implementation
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
 * @addtogroup APP_ANPS_TASK
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

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_anps_env_tag *app_anps_env = &app_env.anps_ev;

/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the ANPS.     *//**
 *
 * @param[in] msgid     ANPS_DISABLE_IND
 * @param[in] param     Pointer to the struct anps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used by the ANP Server role to inform the application of a correct
 * disable after a disconnection.
 * 
 * The current notification configuration values for the New Alert characteristic and the
 * Unread Alert Status characteristic are included in the parameters so that the higher 
 * application may safely keep the configuration until the next time the profile role is
 * enabled.
 ****************************************************************************************
 */
int app_anps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct anps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_anps_env->conhdl = 0xFFFF;
    app_anps_env->enabled = false;
    
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles requests to be notified about new alert values.       *//**
 *
 * @param[in] msgid     ANPS_NTF_IMMEDIATE_REQ_IND
 * @param[in] param     Pointer to the struct anps_ntf_immediate_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * 
 * This handler is sent to the application when the Alert Notification Control Point 
 * Charactersitic value is written by the peer device with a ¡®Notify New Incoming Alert
 * Immediately¡¯ or ¡®Notify Unread Category Status Immediately¡¯ command id.
 * 
 * The cat_ntf_cfg parameter provided information about the categories that has been 
 * required by the peer device and which can be notified (supported + enabled).
 * 
 * If there are no new alerts or no unread alerts for the specified category, the provided
 * number of alert shall be set to 0.
 ****************************************************************************************
 */
int app_anps_ntf_immediate_req_ind_handler(ke_msg_id_t const msgid,
                                           struct anps_ntf_immediate_req_ind *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles Indicate that the notification configuration has been modified by the 
 *        peer device.       *//**
 *
 * @param[in] msgid     ANPS_NTF_STATUS_UPDATE_IND
 * @param[in] param     Pointer to the struct anps_ntf_status_update_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * 
 * This handler is sent to the application when the value of one of the two Client Characteristic
 * Configuration descriptors has been written by the peer device to enable or diable sending of
 * notifications. It is also sent upon reception of a write request for the Alert Notification
 * Control Point characteristic with the following command ids: 
 *    -  Enable New Incoming Alert Notifications 
 *    -  Enable Unread Category Status Notifications 
 *    -  Disable New Incoming Alert Notifications 
 *    -  Disable Unread Category Status Notifications
 ****************************************************************************************
 */
int app_anps_ntf_status_update_ind_handler(ke_msg_id_t const msgid,
                                           struct anps_ntf_status_update_ind *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    if (param->alert_type == ANP_NEW_ALERT)
    {
        if (param->ntf_ccc_cfg == PRF_CLI_START_NTF)
            app_anps_env->ntf_cfg |= 1 << ANP_NEW_ALERT;
        else
            app_anps_env->ntf_cfg &= ~(1 << ANP_NEW_ALERT);
    }
    else if (param->alert_type == ANP_UNREAD_ALERT)
    {
        if (param->ntf_ccc_cfg == PRF_CLI_START_NTF)
            app_anps_env->ntf_cfg |= 1 << ANP_UNREAD_ALERT;
        else
            app_anps_env->ntf_cfg &= ~(1 << ANP_UNREAD_ALERT);
    }

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles Complete Event Information.       *//**
 *
 * @param[in] msgid     ANPS_CMP_EVT
 * @param[in] param     Pointer to the struct anps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * 
 * The handler is used by the ANPS task to inform the sender of a command that the procedure
 * is over and contains the status of the procedure. 
 ****************************************************************************************
 */
int app_anps_cmp_evt_handler(ke_msg_id_t const msgid,
                             struct anps_cmp_evt *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        switch (param->operation)
        {
            case ANPS_CREATE_DB_OP_CODE:
                if (param->status == ATT_ERR_NO_ERROR)
                {
                    app_clear_local_service_flag(BLE_AN_SERVER_BIT);
                }
                break;
            case ANPS_ENABLE_OP_CODE:
                break;
            case ANPS_UPD_NEW_ALERT_OP_CODE:
                break;
            case ANPS_UPD_UNREAD_ALERT_STATUS_OP_CODE:
                break;
            default:
                break;
        }
    }
    
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_AN_SERVER

/// @} APP_ANPS_TASK
