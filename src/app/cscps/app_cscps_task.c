/**
 ****************************************************************************************
 *
 * @file app_cscps_task.c
 *
 * @brief Application CSCPS implementation
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
 * @addtogroup APP_CSCPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#if BLE_CSC_SENSOR
#include "app_cscps.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_cscps_env_tag *app_cscps_env = &app_env.cscps_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the CSCPS.      *//**
 *
 * @param[in] msgid     CSCPS_DISABLE_IND
 * @param[in] param     Pointer to the struct cscps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The current notification
 * configuration values for the Measurement characteristic and the SC control point characteristic
 * are included in the parameters so that the higher application may safely keep the
 * configuration until the next time the profile role is enabled.
 ****************************************************************************************
 */
int app_cscps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct cscps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_cscps_env->conhdl = 0xffff;
    app_cscps_env->enabled = false;
    app_cscps_env->app_cfg |= (param->csc_meas_ntf_cfg == PRF_CLI_START_NTF) ? CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF : 0;
    app_cscps_env->app_cfg |= (param->sc_ctnl_pt_ntf_cfg == PRF_CLI_START_IND) ? CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND : 0;
    app_cscps_env->wheel_revol = param->wheel_revol;
    app_cscps_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the control point indication from the CSCPS.      *//**
 *
 * @param[in] msgid     CSCPS_SC_CTNL_PT_REQ_IND
 * @param[in] param     Pointer to the struct cscps_sc_ctnl_pt_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of the control point characteristic is
 * written by the peer device.
 ****************************************************************************************
 */
int app_cscps_ctnl_pt_req_ind_handler(ke_msg_id_t const msgid,
                                      struct cscps_sc_ctnl_pt_req_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the CSCPS.      *//**
 *
 * @param[in] msgid     CSCPS_NTF_IND_CFG_IND
 * @param[in] param     Pointer to the struct cscps_ntf_ind_cfg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 ****************************************************************************************
 */
int app_cscps_ntf_ind_cfg_ind_handler(ke_msg_id_t const msgid,
                                      struct cscps_ntf_ind_cfg_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (param->char_code == CSCP_CSCS_CSC_MEAS_CHAR)
    {
        if (param->ntf_cfg == PRF_CLI_START_NTF)
            app_cscps_env->app_cfg |= CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF;
        else
            app_cscps_env->app_cfg &= ~CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF;
    }
    else if (param->char_code == CSCP_CSCS_SC_CTNL_PT_CHAR)
    {
        if (param->ntf_cfg == PRF_CLI_START_IND)
            app_cscps_env->app_cfg |= CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND;
        else
            app_cscps_env->app_cfg &= ~CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND;
    }

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the command complete event from the CSCPS.      *//**
 *
 * @param[in] msgid     CSCPS_CMP_EVT
 * @param[in] param     Pointer to the struct cscps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the sender of a command that the procedure is over and
 * contains the status of the procedure.
 ****************************************************************************************
 */
int app_cscps_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct cscps_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case CSCPS_CREATE_DB_OP_CODE:
            if (param->status == ATT_ERR_NO_ERROR)
                app_clear_local_service_flag(BLE_CSCP_SERVER_BIT);
            break;
        case CSCPS_ENABLE_OP_CODE:
            break;
        case CSCPS_SEND_CSC_MEAS_OP_CODE:
            app_cscps_env->ntf_sending = false;
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

#endif // BLE_CSC_SENSOR

/// @} APP_CSCPS_TASK
