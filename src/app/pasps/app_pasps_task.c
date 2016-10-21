/**
 ****************************************************************************************
 *
 * @file app_pasps_task.c
 *
 * @brief Application PASPS implementation
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
 * @addtogroup APP_PASPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_PAS_SERVER
#include "app_pasps.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_pasps_env_tag *app_pasps_env = &app_env.pasps_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the PASPS.      *//**
 *
 * @param[in] msgid     PASPS_DISABLE_IND
 * @param[in] param     Pointer to the struct pasps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The current notification
 * configuration values for the Alert status characteristic and the Ringer Setting characteristic
 * are included in the parameters so that the higher application may safely keep the
 * configuration until the next time the profile role is enabled.
 ****************************************************************************************
 */
int app_pasps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct pasps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_pasps_env->conhdl = 0xffff;
    app_pasps_env->enabled = false;
    app_pasps_env->app_cfg |= (param->alert_status_ntf_cfg == PRF_CLI_START_NTF) ? PASPS_FLAG_ALERT_STATUS_CFG : 0;
    app_pasps_env->app_cfg |= (param->ringer_setting_ntf_cfg == PRF_CLI_START_IND) ? PASPS_FLAG_RINGER_SETTING_CFG : 0;
    app_pasps_env->ntf_sending = 0;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the written characteristic indication from the PASPS.      *//**
 *
 * @param[in] msgid     PASPS_WRITTEN_CHAR_VAL_IND
 * @param[in] param     Pointer to the struct pasps_written_char_val_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application that the value of one of the writable
 * attribute has been successfully written by the peer device.
 *
 ****************************************************************************************
 */
int app_pasps_written_char_val_ind_handler(ke_msg_id_t const msgid,
                                           struct pasps_written_char_val_ind *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    switch (param->att_code)
    {
        case PASPS_RINGER_CTNL_PT_CHAR_VAL:
            break;
        case PASPS_ALERT_STATUS_NTF_CFG:
            if (param->value.alert_status_ntf_cfg == PRF_CLI_START_NTF)
                app_pasps_env->app_cfg |= PASPS_FLAG_ALERT_STATUS_CFG;
            else
                app_pasps_env->app_cfg &= ~PASPS_FLAG_ALERT_STATUS_CFG;
            break;
        case PASPS_RINGER_SETTING_NTF_CFG:
            if (param->value.ringer_setting_ntf_cfg == PRF_CLI_START_NTF)
                app_pasps_env->app_cfg |= PASPS_FLAG_RINGER_SETTING_CFG;
            else
                app_pasps_env->app_cfg &= ~PASPS_FLAG_RINGER_SETTING_CFG;
            break;
    }

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the command complete event from the PASPS.      *//**
 *
 * @param[in] msgid     PASPS_CMP_EVT
 * @param[in] param     Pointer to the struct pasps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the sender of a command that the procedure is over and
 * contains the status of the procedure.
 ****************************************************************************************
 */
int app_pasps_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct pasps_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case PASPS_CREATE_DB_OP_CODE:
            if (param->status == ATT_ERR_NO_ERROR)
            {
                app_clear_local_service_flag(BLE_PASP_SERVER_BIT);
            }
            break;
        case PASPS_ENABLE_OP_CODE:
            break;
        case PASPS_UPD_ALERT_STATUS_OP_CODE:
            app_pasps_env->ntf_sending |= PASPS_FLAG_ALERT_STATUS_CFG;
            break;
        case PASPS_UPD_RINGER_SETTING_OP_CODE:
            app_pasps_env->ntf_sending |= PASPS_FLAG_RINGER_SETTING_CFG;
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

#endif // BLE_PAS_SERVER

/// @} APP_PASPS_TASK
