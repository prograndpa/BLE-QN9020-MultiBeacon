/**
 ****************************************************************************************
 *
 * @file app_cscpc_task.c
 *
 * @brief Application CSCPC implementation
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
 * @addtogroup APP_CSCPC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_CSC_COLLECTOR
#include "app_cscpc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_cscpc_env_tag *app_cscpc_env = &app_env.cscpc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 

/*
 ****************************************************************************************
 * @brief Handles the enable result from TASK_CSCPC. *//**
 *
 * @param[in] msgid     CSCPC_CSCS_CONTENT_IND
 * @param[in] param     Pointer to struct cscpc_cscs_content_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the discovery results of 
 * Cycling Speed and Cadence Profile Sensor.
 *
 ****************************************************************************************
 */
int app_cscpc_cscs_content_ind_handler(ke_msg_id_t const msgid,
                                       struct cscpc_cscs_content_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    app_cscpc_env[idx].conhdl = param->conhdl;
    app_cscpc_env[idx].enabled = true;
    //app_cscpc_env[app_env.select_idx].cscs = param->cscs;
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the value indication from TASK_CSCPC. *//**
 *
 * @param[in] msgid     CSCPC_VALUE_IND
 * @param[in] param     Pointer to struct cscpc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that a new value is received from the peer device
 * within a read response, an indication, or a notificatuon.
 *
 ****************************************************************************************
 */
int app_cscpc_value_ind_handler(ke_msg_id_t const msgid,
                                struct cscpc_value_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch (param->att_code)
    {
        case CSCPC_NTF_CSC_MEAS:
            if (param->value.csc_meas.flags & CSCP_MEAS_WHEEL_REV_DATA_PRESENT)
            {
                QPRINTF("Cumulative Wheel %#x, Last Wheel Event Time %#02x\r\n", 
                  param->value.csc_meas.cumul_wheel_rev, param->value.csc_meas.last_wheel_evt_time);
            }
            if (param->value.csc_meas.flags & CSCP_MEAS_CRANK_REV_DATA_PRESENT)
            {
                QPRINTF("Cumulative Crank %#02x, Last Crank Event Time %#02x\r\n", 
                  param->value.csc_meas.cumul_crank_rev, param->value.csc_meas.last_crank_evt_time);
            }
            break;
        case CSCPC_RD_CSC_FEAT:
            QPRINTF("Sensor Feature is %d\r\n", param->value.sensor_feat);
            break;
        case CSCPC_RD_SENSOR_LOC:
            QPRINTF("Sensor Location is %d\r\n", param->value.sensor_loc);
            break;
        case CSCPC_RD_WR_CSC_MEAS_CFG:
        case CSCPC_RD_WR_SC_CTNL_PT_CFG:
            QPRINTF("CCC Value is %d\r\n", param->value.ntf_cfg);
            break;
        case CSCPC_IND_SC_CTNL_PT:
            if (param->value.ctnl_pt_rsp.req_op_code == CSCP_CTNL_PT_OP_REQ_SUPP_LOC)
            {
                if (param->value.ctnl_pt_rsp.resp_value == CSCP_CTNL_PT_RESP_SUCCESS)
                    QPRINTF("Supported locations is %#x\r\n", param->value.ctnl_pt_rsp.supp_loc);
                else
                    QPRINTF("Control point result is %#x\r\n", param->value.ctnl_pt_rsp.resp_value);
            }
            else
            {
                QPRINTF("Control point result is %#x\r\n", param->value.ctnl_pt_rsp.resp_value);
            }
            break;
        default:
            break;
    }

    switch (param->att_code)
    {
        case CSCPC_RD_CSC_FEAT:
        {
            QPRINTF("CSC Feature: 0x%04x\r\n", param->value.sensor_feat);
        }
            break;
        case CSCPC_RD_SENSOR_LOC:
        {
            QPRINTF("Sensor Location: 0x%02x\r\n", param->value.sensor_loc);
        }
            break;
        default:
            break;
    }
    app_task_msg_hdl(msgid, param);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the complete event from TASK_CSCPC. *//**
 *
 * @param[in] msgid     CSCPC_CMP_EVT
 * @param[in] param     Pointer to struct cscpc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the sender of a command that the procedure is over and 
 * contains the status of the procedure.
 *
 ****************************************************************************************
 */
int app_cscpc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct cscpc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case CSCPC_ENABLE_OP_CODE:
            if (param->status == PRF_ERR_OK)
            {
                QPRINTF("Enable complete\r\n");
            }
            else
            {
                QPRINTF("Enable failed %d\r\n", param->status);
            }
            break;
        case CSCPC_READ_OP_CODE:
            QPRINTF("Read procedure complete, result is %d\r\n", param->status);
            break;
        case CSCPC_CFG_NTF_IND_OP_CODE:
            QPRINTF("Configure procedure complete, result is %d\r\n", param->status);
            break;
        case CSCPC_CTNL_PT_CFG_WR_OP_CODE:
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
 * @param[in] msgid     CSCPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Cycling Speed and Cadence Profile
 * Collector Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_cscpc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

#endif // BLE_CSC_COLLECTOR

/// @} APP_CSCPC_TASK
