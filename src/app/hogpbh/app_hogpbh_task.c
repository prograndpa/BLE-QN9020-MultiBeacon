/**
 ****************************************************************************************
 *
 * @file app_hogpbh_task.c
 *
 * @brief Application HOGPBH task implementation
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
 * @addtogroup APP_HOGPBH_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HID_BOOT_HOST
#include "app_hogpbh.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_hogpbh_env_tag *app_hogpbh_env = &app_env.hogpbh_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HOGPBH. *//**
 *
 * @param[in] msgid     HOGPBH_ENABLE_CFM
 * @param[in] param     Pointer to the struct hogpbh_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Boot Host to either send the discovery results of HIDS on the HID device 
 *  and confirm enabling of the Boot Host role, or to simply confirm enabling of Boot Host role if it is a normal connection 
 *  and the attribute details are already known. 
 *
 ****************************************************************************************
 */
int app_hogpbh_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct hogpbh_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPBH enable confirmation status: 0x%X.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR)
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_hogpbh_env[idx].conhdl = param->conhdl;
        app_hogpbh_env[idx].enabled = true;
        app_hogpbh_env[idx].cur_code = 0;

        // Get keyboard instance number here
        if (param->hids[0].descs[HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG].desc_hdl != 0)
            app_hogpbh_env[idx].hids_kb = 0;
        else if (param->hids[1].descs[HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG].desc_hdl != 0)
            app_hogpbh_env[idx].hids_kb = 1;
        else
            app_hogpbh_env[idx].hids_kb = 0xFF;

        // Get mouse instance number here
        if (param->hids[0].descs[HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG].desc_hdl != 0)
            app_hogpbh_env[idx].hids_mouse = 0;
        else if (param->hids[1].descs[HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG].desc_hdl != 0)
            app_hogpbh_env[idx].hids_mouse = 1;
        else
            app_hogpbh_env[idx].hids_mouse = 0xFF;

        if (app_hogpbh_env[idx].hids_kb < HOGPBH_NB_HIDS_INST_MAX)
        {
            // Set Keyboard to Boot report mode
            app_hogpbh_set_boot_proto_mode_req(app_hogpbh_env[idx].hids_kb, param->conhdl);
            // Start Keybaord Notify here
            app_hogpbh_cfg_ntf_req(HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG, PRF_CLI_START_NTF, 
                                    app_hogpbh_env[idx].hids_kb, param->conhdl);
            app_hogpbh_env[idx].cur_code = 1;
        }
        
        if (app_hogpbh_env[idx].hids_mouse != app_hogpbh_env[idx].hids_kb
         && app_hogpbh_env[idx].hids_mouse  < HOGPBH_NB_HIDS_INST_MAX)
        {
            // Set Mouse to Boot protocol mode
            app_hogpbh_set_boot_proto_mode_req(app_hogpbh_env[idx].hids_mouse, param->conhdl);
            if (!app_hogpbh_env[idx].cur_code) {
                // Start Mouse Notify here
                app_hogpbh_cfg_ntf_req(HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG, PRF_CLI_START_NTF, 
                                        app_hogpbh_env[idx].hids_mouse, param->conhdl);
            }
            else {
                app_hogpbh_env[idx].cur_code = 2;
            }
        }
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     HOGPBH_WR_CHAR_RSP
 * @param[in] param     Pointer to the struct hogpbh_char_req_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  The API is used to inform the application about the status of the writing request that has been 
 *  sent. 
 *
 ****************************************************************************************
 */
int app_hogpbh_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hogpbh_char_req_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPBH write char response status: 0x%X.\r\n", param->status);
    switch (param->att_code) {
    case HOGPBH_RD_WR_HIDS_PROTO_MODE:
        break;
    case HOGPBH_RD_WR_HIDS_BOOT_KB_OUT_REPORT:
        break;
    default:
        break;
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for send read notification configuration value to APP. *//**
 *
 * @param[in] msgid     HOGPBH_CFG_NTF_RD_RSP 
 * @param[in] param     Pointer to the struct hogpbh_cfg_ntf_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read Client Characteristic Configuration 
 *  Descriptor value. 
 *
 ****************************************************************************************
 */
int app_hogpbh_cfg_ntf_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogpbh_cfg_ntf_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPBH read configuration response(%d).\r\n", param->hids_nb);
   
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for send Read Protocol Mode value to APP. *//**
 *
 * @param[in] msgid     HOGPBH_PROTO_MODE_RD_RSP 
 * @param[in] param     Pointer to the struct hogpbh_proto_mode_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read Protocol Mode Characteristic value. 
 *
 ****************************************************************************************
 */
int app_hogpbh_proto_mode_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogpbh_proto_mode_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPBH protocol mode read response mode(%d): ", param->hids_nb);

    switch (param->proto_mode)
    {
    case HOGP_BOOT_PROTOCOL_MODE:
        QPRINTF("Boot mode.\r\n");
        break;
    case HOGP_REPORT_PROTOCOL_MODE:
        QPRINTF("Report mode.\r\n");
        break;
    default:
        QPRINTF("NA.\r\n");
        break;
    }
 
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message Boot Report value send to APP. (after Read Request or Notification) *//**
 *
 * @param[in] msgid     HOGPBH_BOOT_REPORT_IND
 * @param[in] param     Pointer to the struct hogpbh_boot_report_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API  is used to inform the application about the read Boot Keyboard Input Report 
 *  Characteristic value. 
 *
 ****************************************************************************************
 */
int app_hogpbh_boot_report_ind_handler(ke_msg_id_t const msgid,
                                       struct hogpbh_boot_report_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    switch (param->ind_type)
    {
    case HOGPBH_IND_RD_RSP:
        break;
    case HOGPBH_IND_NTF:
        if (param->char_code == HOGPBH_CHAR_BOOT_KB_IN_REPORT) {
            QPRINTF("HOGPBH Keyboard in report notification(%d):\r\n", param->hids_nb);
            QTRACE(param->report, param->report_length, 0, 2);
            QPRINTF("\r\n");
            if (app_hogpbh_env[idx].cur_code == 2) {
                // Start Mouse Notify here
                app_hogpbh_cfg_ntf_req(HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG, PRF_CLI_START_NTF, 
                                        app_hogpbh_env[idx].hids_mouse, param->conhdl);
                app_hogpbh_env[idx].cur_code = 0;
            }
        }
        else if (param->char_code == HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT) {
            QPRINTF("HOGPBH Mouse in report notification(%d):\r\n", param->hids_nb);
            QTRACE(param->report, param->report_length, 0, 2);
            QPRINTF("\r\n");
        }
        break;
    default:
        break;
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles read request response message from the HOGPBH. *//**
 *
 * @param[in] msgid     HOGPBH_RD_CHAR_ERR_RSP 
 * @param[in] param     Pointer to the struct hogpbh_char_req_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called when application sent a read request is not complient with the 
 *  specification or the implementation limitations.
 *
 ****************************************************************************************
 */
int app_hogpbh_char_req_rsp_handler(ke_msg_id_t const msgid,
                                    struct hogpbh_char_req_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("HOGPBH read char error response\r\n");
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     HOGPBH_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the HID Over GATT Profile Boot Host 
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_hogpbh_disable_ind_handler(ke_msg_id_t const msgid,
                                   struct prf_client_disable_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("HOGPBH disable ind\r\n");
    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_HOGPBH_TASK
