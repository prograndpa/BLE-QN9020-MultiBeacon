/**
 ****************************************************************************************
 *
 * @file app_rscpc_task.c
 *
 * @brief Application RSCPC implementation
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
 * @addtogroup APP_RSCPC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_RSC_COLLECTOR
#include "app_rscpc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_rscpc_env_tag *app_rscpc_env = &app_env.rscpc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the result of discovery procedure from TASK_RSCPC. *//**
 *
 * @param[in] msgid     RSCPC_RSCS_CONTENT_IND
 * @param[in] param     Pointer to struct rscpc_rscs_content_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the result of discovery
 * Running Speed and Cadence Profile Sensor. It contains the structure of the peer
 * device RSCS.
 *
 ****************************************************************************************
 */
int app_rscpc_rscs_content_ind_handler(ke_msg_id_t const msgid,
                                       struct rscpc_rscs_content_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    app_rscpc_env[idx].conhdl = param->conhdl;
    app_rscpc_env[idx].enabled = true;
    //app_rscpc_env[app_env.select_idx].rscs = param->rscs;
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the value indication from TASK_RSCPC. *//**
 *
 * @param[in] msgid     RSCPC_VALUE_IND
 * @param[in] param     Pointer to struct rscpc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that a new value is received from the peer device
 * within a read response, an indication, or a notificatuon.
 *
 ****************************************************************************************
 */
int app_rscpc_value_ind_handler(ke_msg_id_t const msgid,
                                struct rscpc_value_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch (param->att_code)
    {
        case RSCPC_NTF_RSC_MEAS:
            QPRINTF("RSC Measurement %d\r\n", param->value.rsc_meas.inst_cad);
            break;
        case RSCPC_RD_RSC_FEAT:
            QPRINTF("Feature is %d\r\n", param->value.sensor_feat);
            break;
        case RSCPC_RD_SENSOR_LOC:
            QPRINTF("Sensor loc is %#x\r\n", param->value.sensor_loc);
            break;
        case RSCPC_IND_SC_CTNL_PT:
        {
            QPRINTF("Req Op is %d ", param->value.ctnl_pt_rsp.req_op_code);
            if (param->value.ctnl_pt_rsp.req_op_code == RSCP_CTNL_PT_OP_REQ_SUPP_LOC)
            {
                if (param->value.ctnl_pt_rsp.resp_value == RSCP_CTNL_PT_RESP_SUCCESS)
                    QPRINTF("Sensor loc is %#x\r\n", param->value.ctnl_pt_rsp.supp_loc);
                else
                    QPRINTF("Request loc failed %d\r\n", param->value.ctnl_pt_rsp.resp_value);
            }
            else
            {
                QPRINTF("Resp value is %d\r\n", param->value.ctnl_pt_rsp.resp_value);
            }
            break;
        }
        case RSCPC_RD_WR_RSC_MEAS_CFG:
        case RSCPC_RD_WR_SC_CTNL_PT_CFG:
        {
            switch (param->value.ctnl_pt_rsp.req_op_code)
            {
                case RSCP_CTNL_PT_OP_SET_CUMUL_VAL:
                    QPRINTF("Set Cumulative Value %d,", param->value.ctnl_pt_rsp.resp_value);
                    break;
                case RSCP_CTNL_PT_OP_START_CALIB:
                    QPRINTF("Start Sensor Calibration %d,", param->value.ctnl_pt_rsp.resp_value);
                    break;
                case RSCP_CTNL_PT_OP_UPD_LOC:
                    QPRINTF("Update Sensor Location %d,", param->value.ctnl_pt_rsp.resp_value);
                    break;
                case RSCP_CTNL_PT_OP_REQ_SUPP_LOC:
                    QPRINTF("Request Supported Sensor Locations %d,", param->value.ctnl_pt_rsp.resp_value);
                    break;
                default:
                    break;
            }
        }
            QPRINTF("Read CCC value is %d\r\n", param->value.ntf_cfg);
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the complete event from TASK_RSCPC. *//**
 *
 * @param[in] msgid     RSCPC_CMP_EVT
 * @param[in] param     Pointer to struct rscpc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the sender of a command that the procedure is over and 
 * contains the status of the procedure.
 *
 ****************************************************************************************
 */
int app_rscpc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct rscpc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case RSCPC_ENABLE_OP_CODE:
            if (param->status == PRF_ERR_OK)
            {
                QPRINTF("Enable complete\r\n");
            }
            else
            {
                QPRINTF("Enable failed %d\r\n", param->status);
            }
            break;
        case RSCPC_READ_OP_CODE:
            QPRINTF("Read procedure complete, result is %d\r\n", param->status);
            break;
        case RSCPC_CFG_NTF_IND_OP_CODE:
            QPRINTF("Configure procedure complete, result is %d\r\n", param->status);
            break;
        case RSCPC_CTNL_PT_CFG_WR_OP_CODE:
            QPRINTF("Ctrl procedure complete, result is %d\r\n", param->status);
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     RSCPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Running Speed and Cadence Profile
 * Collector Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_rscpc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

#endif // BLE_RSC_COLLECTOR

/// @} APP_RSCPC_TASK

