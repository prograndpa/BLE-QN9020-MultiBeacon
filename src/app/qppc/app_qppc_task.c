/**
 ****************************************************************************************
 *
 * @file app_qppc_task.c
 *
 * @brief Application QPPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_QPP_CLIENT
#include "app_qppc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_qppc_env_tag *app_qppc_env = &app_env.qppc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the QPPC. *//**
 *
 * @param[in] msgid     QPPC_ENABLE_CFM
 * @param[in] param     Pointer to struct qppc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client to either send the discovery results of QPPS and 
 * confirm enabling of the Client role, or to simply confirm enabling of Client 
 * role if it is a normal connection and the attribute details are already known.  
 *
 ****************************************************************************************
 */
int app_qppc_enable_cfm_handler(ke_msg_id_t const msgid,
                                struct qppc_enable_cfm *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("QPPC enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_qppc_env[idx].conhdl = param->conhdl;
        app_qppc_env[idx].enabled = true;
        app_qppc_env[idx].cur_code = QPPC_QPPS_RX_CHAR_VALUE_USER_DESP;
        app_qppc_env[idx].nb_ntf_char = param->nb_ntf_char;
        app_qppc_rd_char_req(app_qppc_env[idx].conhdl, QPPC_QPPS_RX_CHAR_VALUE_USER_DESP);
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the QPPC. *//**
 *
 * @param[in] msgid     QPPC_ERROR_IND
 * @param[in] param     Pointer to struct qppc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This handler is called when an error has been raised in the QPP Client role task. 
 *
 ****************************************************************************************
 */
int app_qppc_error_ind_handler(ke_msg_id_t const msgid,
                               struct qppc_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("QPPC error indication status: 0x%x, cur_code is: %d\r\n", param->status, app_qppc_env->cur_code);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     QPPC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct qppc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received read response. The 
 * status and the data from the read response are passed directly to Application, which must interpret 
 * them based on the request it made.
 *
 ****************************************************************************************
 */
int app_qppc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                                 struct qppc_rd_char_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    if (app_qppc_env[idx].cur_code == QPPC_QPPS_RX_CHAR_VALUE_USER_DESP)
    {
        app_qppc_cfg_indntf_req(PRF_CLI_START_NTF, app_qppc_env[idx].conhdl, QPPC_QPPS_FIRST_TX_VALUE_CLI_CFG);
        app_qppc_env[idx].cur_code = QPPC_QPPS_FIRST_TX_VALUE_CLI_CFG;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP.  *//**
 *
 * @param[in] msgid     QPPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct qppc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received write response.  
 * The status and the data from the write response are passed directly to Application, which 
 * must interpret them based on the request it made.
 *
 ****************************************************************************************
 */
int app_qppc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                                 struct qppc_wr_char_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        if (app_qppc_env[idx].cur_code < app_qppc_env[idx].nb_ntf_char)
        {
            app_qppc_cfg_indntf_req(PRF_CLI_START_NTF, app_qppc_env[idx].conhdl, ++app_qppc_env[idx].cur_code);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the value send to APP. *//**
 *
 * @param[in] msgid     QPPC_DATA_IND
 * @param[in] param     Pointer to struct qppc_data_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received value by 
 * notification. The application will do what it needs to do with the received value. 
 *
 ****************************************************************************************
 */
uint8_t data_pack = 0;
int app_qppc_data_ind_handler(ke_msg_id_t const msgid,
                              struct qppc_data_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    if ((data_pack%200) == 0 && (data_pack != 0))
    {
        QPRINTF("(%d)Received %d bytes data[%02X] from characteristic %d\r\n", idx, param->length, param->data[0], param->char_code);
    }
    app_task_msg_hdl(msgid, param);
    data_pack ++;

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the QPPC disable indication. *//**
 *
 * @param[in] msgid     QPPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application that the Client Role task has been
 * correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_qppc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    QPRINTF("idx = %d\r\n", idx);
    app_qppc_env[idx].conhdl = 0xFFFF;
    app_qppc_env[idx].enabled = false;
    app_qppc_env[idx].nb_ntf_char = 0;

    return (KE_MSG_CONSUMED);
}

#endif

