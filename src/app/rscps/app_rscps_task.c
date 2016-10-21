/**
 ****************************************************************************************
 *
 * @file app_rscps_task.c
 *
 * @brief Application RSCPS implementation
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
 * @addtogroup APP_RSCPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_RSC_SENSOR
#include "app_rscps.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_rscps_env_tag *app_rscps_env = &app_env.rscps_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the RSCPS.      *//**
 *
 * @param[in] msgid     RSCPS_DISABLE_IND
 * @param[in] param     Pointer to the struct rscps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The current notification
 * configuration values for the Measurement characteristic and the SC control point characteristic
 * are included in the parameters so that the higher application may safely keep the
 * configuration until the next time the profile role is enabled.
 ****************************************************************************************
 */
int app_rscps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct rscps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_rscps_env->conhdl = 0xffff;
    app_rscps_env->enabled = false;
    app_rscps_env->app_cfg |= (param->rsc_meas_ntf_cfg == PRF_CLI_START_NTF) ? RSCP_PRF_CFG_FLAG_RSC_MEAS_NTF : 0;
    app_rscps_env->app_cfg |= (param->sc_ctnl_pt_ntf_cfg == PRF_CLI_START_IND) ? RSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND : 0;
    app_rscps_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the control point indication from the RSCPS.      *//**
 *
 * @param[in] msgid     RSCPS_SC_CTNL_PT_REQ_IND
 * @param[in] param     Pointer to the struct rscps_sc_ctnl_pt_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of the control point characteristic is
 * written by the peer device.
 ****************************************************************************************
 */
int app_rscps_ctnl_pt_req_ind_handler(ke_msg_id_t const msgid,
                                      struct rscps_sc_ctnl_pt_req_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the RSCPS.      *//**
 *
 * @param[in] msgid     RSCPS_NTF_IND_CFG_IND
 * @param[in] param     Pointer to the struct rscps_ntf_ind_cfg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 ****************************************************************************************
 */
int app_rscps_ntf_ind_cfg_ind_handler(ke_msg_id_t const msgid,
                                      struct rscps_ntf_ind_cfg_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (param->char_code == RSCP_RSCS_RSC_MEAS_CHAR)
    {
        if (param->ntf_cfg == PRF_CLI_START_NTF)
            app_rscps_env->app_cfg |= RSCP_PRF_CFG_FLAG_RSC_MEAS_NTF;
        else
            app_rscps_env->app_cfg &= ~RSCP_PRF_CFG_FLAG_RSC_MEAS_NTF;
    }
    else if (param->char_code == RSCP_RSCS_SC_CTNL_PT_CHAR)
    {
        if (param->ntf_cfg == PRF_CLI_START_IND)
            app_rscps_env->app_cfg |= RSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND;
        else
            app_rscps_env->app_cfg &= ~RSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND;
    }

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the command complete event from the RSCPS.      *//**
 *
 * @param[in] msgid     RSCPS_CMP_EVT
 * @param[in] param     Pointer to the struct rscps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the sender of a command that the procedure is over and
 * contains the status of the procedure.
 ****************************************************************************************
 */
int app_rscps_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct rscps_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case RSCPS_CREATE_DB_OP_CODE:
            if (param->status == ATT_ERR_NO_ERROR)
            {
                app_clear_local_service_flag(BLE_RSCP_SERVER_BIT);
            }
            break;
        case RSCPS_SEND_RSC_MEAS_OP_CODE:
            app_rscps_env->ntf_sending = false;
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

#endif // BLE_RSC_SENSOR

/// @} APP_RSCPS_TASK
