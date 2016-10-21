/**
 ****************************************************************************************
 *
 * @file app_basc_task.c
 *
 * @brief Application BASC task implementation
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
 * @addtogroup APP_BASC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_BATT_CLIENT
#include "app_basc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_basc_env_tag *app_basc_env = &app_env.basc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the BASC. *//**
 *
 * @param[in] msgid     BASC_ENABLE_CFM
 * @param[in] param     Pointer to struct basc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role task to either send the discovery results of HID 
 *  on the peer device and confirm enabling of the Client role, or to simply confirm enabling 
 *  of Client role if it is a normal connection and the attribute details are already known.
 *
 ****************************************************************************************
 */
int app_basc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct basc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BASC enable confirmation status: 0x%x.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);

        app_basc_env[idx].conhdl = param->conhdl;
        app_basc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the BASC. *//**
 *
 * @param[in] msgid     BASC_ERROR_IND
 * @param[in] param     Pointer to struct basc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called when an error has been raised in the BASC Client role task. 
 *
 ****************************************************************************************
 */
int app_basc_error_ind_handler(ke_msg_id_t const msgid,
                      struct basc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BASC error indication status: 0x%x.\r\n", param->status);
    // BASS may do not support Presentation Format

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     BASC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct basc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called when a write response has been received from the peer device 
 *  after sending of a write request. 
 *
 ****************************************************************************************
 */
int app_basc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct basc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BASC write char response status: 0x%x.\r\n", param->status);
    //BASS may do not support Notify function

    app_task_msg_hdl(msgid, param);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Inform APP that Battery Level Notification Configuration has been read. *//**
 *
 * @param[in] msgid     BASC_BATT_LEVEL_NTF_CFG_RD_RSP
 * @param[in] param     Pointer to struct basc_batt_level_ntf_cfg_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called to inform APP about the read Client Characteristic Configuration 
 *  Descriptor value for the specified Battery Level Characteristic.
 *
 ****************************************************************************************
 */
int app_basc_batt_level_ntf_cfg_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct basc_batt_level_ntf_cfg_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BASC Notification Configuration Value(%d): 0x%04x\r\n", param->bas_nb, param->ntf_cfg);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Inform APP that Battery Level Characteristic Presentation Format has been read. *//**
 *
 * @param[in] msgid     BASC_BATT_LEVEL_PRES_FORMAT_RD_RSP
 * @param[in] param     Pointer to struct basc_batt_level_pres_format_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called to inform APP about the read Characteristic Presentation Format 
 *  Descriptor value for the specified Battery Level Characteristic. 
 *
 ****************************************************************************************
 */
int app_basc_batt_level_pres_format_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct basc_batt_level_pres_format_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{

    QPRINTF("BASC Characteristic Presentation Format(%d):\r\n", param->bas_nb);
    QPRINTF("Unit: 0x%X, Description: 0x%X, Format: 0x%X, Exponent: 0x%X, Namespace: 0x%X\r\n", 
        param->char_pres_format.unit,
        param->char_pres_format.description,
        param->char_pres_format.format,
        param->char_pres_format.exponent,
        param->char_pres_format.namespace
        );
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Indicate to APP that the Battery Level value has been received. *//**
 *
 * @param[in] msgid     BASC_BATT_LEVEL_IND
 * @param[in] param     Pointer to struct basc_batt_level_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called when a Battery Level Characteristic value has been received 
 *  either upon reception of a notification, or upon reception of the read response. 
 *
 ****************************************************************************************
 */
int app_basc_batt_level_ind_handler(ke_msg_id_t const msgid,
                      struct basc_batt_level_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BASC Battery Level value(%d,%d): %d%.\r\n",
        param->bas_nb,
        param->ind_type,
        param->batt_level
        );
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     BASC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Battery Service Client Role
 * task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_basc_disable_ind_handler(ke_msg_id_t const msgid,
                          struct prf_client_disable_ind *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    QPRINTF("BASC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_BASC_TASK
