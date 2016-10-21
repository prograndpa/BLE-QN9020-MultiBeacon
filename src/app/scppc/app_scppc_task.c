/**
 ****************************************************************************************
 *
 * @file app_scppc_task.c
 *
 * @brief Application SCPPC task implementation
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
 * @addtogroup APP_SCPPC_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_SP_CLIENT
#include "app_scppc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_scppc_env_tag *app_scppc_env = &app_env.scppc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the SCPPC. *//**
 *
 * @param[in] msgid     SCPPC_ENABLE_CFM
 * @param[in] param     Pointer to struct scppc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role task to either send the discovery results of SCPS on the peer 
 *  device and confirm enabling of the Client role, or to simply confirm enabling of Client role if it is a normal connection 
 *  and the attribute details are already known. 
 *
 ****************************************************************************************
 */
int app_scppc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct scppc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("SCPPC enable confirmation status: 0x%X.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_scppc_env[idx].conhdl = param->conhdl;
        app_scppc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the SCPPC. *//**
 *
 * @param[in] msgid     SCPPC_ERROR_IND
 * @param[in] param     Pointer to struct scppc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is called by the application when an error has been raised in the SCPP Client role task. 
 *
 ****************************************************************************************
 */
int app_scppc_error_ind_handler(ke_msg_id_t const msgid,
                      struct scppc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("SCPPC error indication status: 0x%X.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     SCPPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct scppc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is called by the application when a write response has been received from the peer device 
 *  after sending of a write request
 *
 ****************************************************************************************
 */
int app_scppc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct scppc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("SCPPC write char response status: 0x%X.\r\n", param->status);
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Message of Scan Refresh Notification Configuration has been read. *//**
 *
 * @param[in] msgid     SCPPC_SCAN_REFRESH_NTF_CFG_RD_RSP
 * @param[in] param     Pointer to struct scppc_scan_refresh_ntf_cfg_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is called by the application to inform it about the read Client Characteristic Configuration 
 *  Descriptor value for the Scan Refresh Characteristic. 
 *
 ****************************************************************************************
 */
int app_scppc_scan_refresh_ntf_cfg_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct scppc_scan_refresh_ntf_cfg_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("SCPPC Notification Configuration Value: 0x%04x\r\n", param->ntf_cfg);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     SCPPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Client role of the SCPS 
 * has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_scppc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("SCPPC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_SCPPC_TASK
