/**
 ****************************************************************************************
 *
 * @file app_anpc_task.c
 *
 * @brief Application ANPC task implementation
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
 * @addtogroup APP_ANPC_TASK
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

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_anpc_env_tag *app_anpc_env = &app_env.anpc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Indicate to APP that the ANPC value has been received. *//**
 *
 * @param[in] msgid     ANPC_VALUE_IND
 * @param[in] param     Pointer to struct anpc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called once an attribute value has been received from the peer device 
 *  upon a notification or a read response message.
 *
 ****************************************************************************************
 */
int app_anpc_value_ind_handler(ke_msg_id_t const msgid,
                               struct anpc_value_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{

    switch (param->att_code)
    {
        case ANPC_RD_WR_NEW_ALERT_CFG:
             QPRINTF("Read ANS New Alert Client Characteristic Configuration Descriptor: 0x%04x.\r\n",
                      param->value.ntf_cfg);
            break;

        case ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG:
             QPRINTF("Read ANS Unread Alert Status Client Characteristic Configuration Descriptor: 0x%04x.\r\n",
                      param->value.ntf_cfg);
            break;
        
        case ANPC_WR_ALERT_NTF_CTNL_PT:
             QPRINTF(" Write ANS Alert Notification Control Point: 0x%02x.\r\n",
                      param->value.new_alert.cat_id);
            break;

        case ANPC_RD_SUP_NEW_ALERT_CAT:
             QPRINTF("ANS Supported New Alert Category: 0x%04x.\r\n",
                      param->value.supp_cat.cat_id_mask_0 | param->value.supp_cat.cat_id_mask_1);
            break;

        case ANPC_RD_SUP_UNREAD_ALERT_CAT:
             QPRINTF("ANS Supported Unread Alert Category: 0x%04x.\r\n",
                      param->value.supp_cat.cat_id_mask_0 | param->value.supp_cat.cat_id_mask_1);
            break;

        case ANPC_CHAR_NEW_ALERT:
        {
            QPRINTF("New Alert Information String Length: 0x%02x.\r\n",param->value.new_alert.info_str_len);
            QPRINTF("New Alert Category ID: 0x%02x.\r\n",param->value.new_alert.cat_id);
            QPRINTF("New Alert Number of alerts: 0x%02x.\r\n",param->value.new_alert.nb_new_alert);
        }
            break;

        case ANPC_CHAR_UNREAD_ALERT_STATUS:
        {
            QPRINTF("Unread Alert Category ID: 0x%02x.\r\n",param->value.unread_alert.cat_id);
            QPRINTF("Unread Alert Number of alerts: 0x%02x.\r\n",param->value.unread_alert.nb_unread_alert);
        }
            break;

        /*
        case ANPC_CHAR_ALERT_NTF_CTNL_PT:
            QPRINTF("ANPC Client Characteristic Configuration Descriptor Value: 0x%04x.\r\n",
                      param->value.ntf_cfg);
            break;
        */
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Indicate the content of the peer device Alert Notification service. *//**
 *
 * @param[in] msgid     ANPC_ANS_CONTENT_IND
 * @param[in] param     Pointer to struct anpc_ans_content_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called at the end of the discovery procedure performed after the
 *  connection establishment. It contains the structure of the peer device ANS.
 *
 ****************************************************************************************
 */
int app_anpc_content_ind_handler(ke_msg_id_t const msgid,
                                 struct anpc_ans_content_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    
    app_anpc_env[idx].conhdl = param->conhdl;
    app_anpc_env[idx].enabled = true;
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Complete Event Information. *//**
 *
 * @param[in] msgid     ANPC_CMP_EVT
 * @param[in] param     Pointer to struct anpc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is callde by the ANPC task to inform the sender of a command that the 
 *  procedure is over and contains the status of the procedure.
 *
 ****************************************************************************************
 */
int app_anpc_cmp_evt_handler(ke_msg_id_t const msgid,
                                  struct anpc_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("ANPC Complete Status Value: %d%.\r\n",param->status);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        switch (param->operation)
        {
            case ANPC_ENABLE_OP_CODE:
            {
                uint8_t idx = KE_IDX_GET(src_id);

                app_anpc_env[idx].conhdl = param->conhdl;
                app_anpc_env[idx].enabled = true;
                QPRINTF("ANPC enable confirmation status: 0x%X.\r\n", param->status);
            }
                break;
            default:
                break;
        }
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     ANPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Alert Notification Client Role
 * task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_anpc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_BASC_TASK
