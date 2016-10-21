/**
 ****************************************************************************************
 *
 * @file app_paspc_task.c
 *
 * @brief Application PASPC implementation
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
 * @addtogroup APP_PASPC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_PAS_CLIENT
#include "app_paspc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_paspc_env_tag *app_paspc_env = &app_env.paspc_ev[0];
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     PASPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Phone Alert Status Profile 
 * Client role task has been correctly disabled or if an error has occurred during this
 * process.
 *
 ****************************************************************************************
 */
int app_paspc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles Indicate that an attribute value has been received either upon notification
 *        or read response. *//**
 *
 * @param[in] msgid     PASPC_VALUE_IND
 * @param[in] param     Pointer to struct paspc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is sent to the application once an attribute value has been received from
 * the peer device upon a notification or a read response message. The content of the value
 * parameter depends of the attribute code value which defines the attribute that has been
 * updated
 *
 ****************************************************************************************
 */
int app_paspc_val_ind_handler(ke_msg_id_t const msgid,
                              struct paspc_value_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    uint8_t att_code = param->att_code;
    switch (att_code)
    {
        case PASPC_RD_ALERT_STATUS:
            QPRINTF("Alert Status characteristic value: %d%.\r\n",
                     param->value.alert_status);
            break;

        case PASPC_RD_RINGER_SETTING:
            QPRINTF("PAS Ringer Setting: %d%.\r\n",
                     param->value.ringer_setting);
            break;

        case PASPC_RD_WR_ALERT_STATUS_CFG:
            QPRINTF("PAS Alert Status Client Characteristic Configuration Descriptor: %d%.\r\n",
                     param->value.alert_status_ntf_cfg);
            break;

        case PASPC_RD_WR_RINGER_SETTING_CFG:
            QPRINTF("PAS Ringer Setting Client Characteristic Configuration Descriptor: %d%.\r\n",
                     param->value.ringer_setting_ntf_cfg);
            break;

        default:
            break;
    }
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles Indicate the content of the pointer device Phone Alert Status service. *//**
 *
 * @param[in] msgid     PASPC_PASS_CONTENT_IND
 * @param[in] param     Pointer to struct paspc_pass_content
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler message is sent to the application at the end of the discovery procedure
 * performed after the connection establishment. It contains the structure of the peer
 * device PASS. 
 *
 ****************************************************************************************
 */
int app_paspc_pass_content_handler(ke_msg_id_t const msgid,
                                   struct paspc_pass_content_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    app_paspc_env[idx].conhdl = param->conhdl;
    app_paspc_env[idx].enabled = true;
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the complete event from TASK_PASPC. *//**
 *
 * @param[in] msgid     PASPC_CMP_EVT
 * @param[in] param     Pointer to struct paspc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the sender of a command that the procedure is over and 
 * contains the status of the procedure.
 *
 ****************************************************************************************
 */
int app_paspc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct paspc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case PASPC_ENABLE_OP_CODE:
            if (param->status == PRF_ERR_OK)
            {
                QPRINTF("Enable complete\r\n");
            }
            else
            {
                QPRINTF("Enable failed %d\r\n", param->status);
            }
            break;
        case PASPC_READ_OP_CODE:
            QPRINTF("Read procedure complete, result is %d\r\n", param->status);
            break;
        case PASPC_WRITE_OP_CODE:
            QPRINTF("Write procedure complete, result is %d\r\n", param->status);
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

#endif // BLE_PAS_CLIENT

/// @} APP_PASPC_TASK
