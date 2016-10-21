/**
 ****************************************************************************************
 *
 * @file app_glps_task.c
 *
 * @brief Application GLPS implementation
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
 * @addtogroup APP_GLPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"
#if BLE_GL_SENSOR
#include "app_glps.h"

/// @cond

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_glps_env_tag *app_glps_env = &app_env.glps_ev;

/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the GLPS.       *//**
 *
 * @param[in] msgid     GLPS_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct glps_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation. It contains status of database creation.
 ****************************************************************************************
 */
int app_glps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct glps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_GL_SENSOR_BIT);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the GLPS enable confirmation message from the GLPS.   *//**
 *
 * @param[in] msgid     GLPS_ENABLE_CFM
 * @param[in] param     Pointer to the struct glps_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used inform the Application that it has been enabled or not.
 ****************************************************************************************
 */
int app_glps_enable_cfm_handler(ke_msg_id_t const msgid,
                                struct glps_enable_cfm *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if (param->status == PRF_ERR_OK)
    {
        app_glps_env->enabled = true;
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the GLPS.      *//**
 *
 * @param[in] msgid     GLPS_DISABLE_IND
 * @param[in] param     Pointer to the struct glps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable or inform that
 * a disconnection happened (information in status).
 ****************************************************************************************
 */
int app_glps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct glps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_glps_env->conhdl = 0xFFFF;
    app_glps_env->evt_cfg = param->evt_cfg;
    app_glps_env->enabled = false;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the GLPS.     *//**
 *
 * @param[in] msgid     GLPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct glps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is triggered when peer device modify notification/indication configuration 
 * of Glucose Sensor role characteristics. If peer device has been bonded, configuration 
 * that collector has set in GLS attributes (evt_cfg) shall be kept by application in a 
 * non-volatile memory for next time this profile role is enabled.
 * @note Glucose sensor event configuration (notification, indication) configured by
 * peer device (Bonded information)
 * - bit 1: Glucose measurement notifications enabled
 * - bit 2: Glucose measurement context notifications enabled
 * - bit 4: Record Access Control Point (RACP) indications enabled
 ****************************************************************************************
 */
int app_glps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct glps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (param->evt_cfg & GLPS_MEAS_NTF_CFG)
    {
        app_glps_env->evt_cfg |= GLPS_MEAS_NTF_CFG;
    }
    else if (param->evt_cfg & GLPS_MEAS_CTX_NTF_CFG)
    {
        app_glps_env->evt_cfg |= GLPS_MEAS_CTX_NTF_CFG;
    }
    else if (param->evt_cfg & GLPS_RACP_IND_CFG)
    {
        app_glps_env->evt_cfg |= GLPS_RACP_IND_CFG;
    }

    if (app_glps_env->evt_cfg & GLPS_MEAS_NTF_CFG && !(param->evt_cfg & GLPS_MEAS_NTF_CFG))
    {
        app_glps_env->evt_cfg &= ~GLPS_MEAS_NTF_CFG;
    }

    if (app_glps_env->evt_cfg & GLPS_MEAS_CTX_NTF_CFG && !(param->evt_cfg & GLPS_MEAS_CTX_NTF_CFG))
    {
        app_glps_env->evt_cfg &= ~GLPS_MEAS_CTX_NTF_CFG;
    }

    if (app_glps_env->evt_cfg & GLPS_RACP_IND_CFG && !(param->evt_cfg & GLPS_RACP_IND_CFG))
    {
        app_glps_env->evt_cfg &= ~GLPS_RACP_IND_CFG;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Record Access Control Point (RACP) request message from the GLPS. *//**
 *
 * @param[in] msgid     GLPS_RACP_REQ_IND
 * @param[in] param     Pointer to the struct glps_racp_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is triggered when peer collector request to perform a Record Access Control
 * Point (RACP) action.
 * This action could be report glucose measurements, report number of measurement, delete 
 * measurements or abort an on-going operation (see Record Access Control Point (RACP) OP Code).
 * This action contains a filter describing which glucose measurement are concerned by the operation.
 * Possible operations:
 * - GLP_REQ_REP_STRD_RECS: Report stored records
 * - GLP_REQ_REP_NUM_OF_STRD_RECS: Report number of stored records
 * - GLP_REQ_DEL_STRD_RECS: Delete stored records
 * - GLP_REQ_ABORT_OP: Abort on-going operation
 * @note During an on-going operation, any other request from peer device will be automatically
 * refused by Glucose service, except GLP_REQ_ABORT_OP (Abort operation). In that case on-going
 * operation shall be stopped. Finally application shall send response with GLP_REQ_ABORT_OP 
 * op_code and status equals GLP_RSP_SUCCESS.
 ****************************************************************************************
 */
int app_glps_racp_req_ind_handler(ke_msg_id_t const msgid,
                                  struct glps_racp_req_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("RACP op_code %d, operator %d, filter_type %d.\r\n", param->racp_req.op_code, param->racp_req.filter.operator,
                                                              param->racp_req.filter.filter_type);
    app_glps_env->racp_req = param->racp_req;

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the request complete event message from the GLPS.     *//**
 *
 * @param[in] msgid     GLPS_REQ_CMP_EVT
 * @param[in] param     Pointer to the struct glps_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is triggered when a requested action has been performed:
 * GLPS_SEND_MEAS_REQ_NTF_CMP: Glucose measurement notification sent completed
 * GLPS_SEND_RACP_RSP_IND_CMP: Record Access Control Point Response Indication sent completed
 ****************************************************************************************
 */
int app_glps_req_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct glps_req_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    switch (param->request)
    {
        case GLPS_SEND_MEAS_REQ_NTF_CMP:
            break;
        case GLPS_SEND_RACP_RSP_IND_CMP:
            break;
        default:
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the GLPS.     *//**
 *
 * @param[in] msgid     GLPS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report an occurred error.
 ****************************************************************************************
 */
int app_glps_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("glps error indication.\r\n");
    return (KE_MSG_CONSUMED);
}

#endif // BLE_GL_SENSOR

/// @} APP_GLPS_TASK
