/**
 ****************************************************************************************
 *
 * @file app_hogprh_task.c
 *
 * @brief Application HOGPRH task implementation
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
 * @addtogroup APP_HOGPRH_TASK 
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HID_REPORT_HOST
#include "app_hogprh.h"


/// @cond

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_hogprh_env_tag *app_hogprh_env = &app_env.hogprh_ev[0];
/// @endcond


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HOGPRH. *//**
 *
 * @param[in] msgid     HOGPRH_ENABLE_CFM  
 * @param[in] param     Pointer to the struct hogprh_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Report Host to either send the discovery results of HIDS on the HID 
 *  device and confirm enabling of the Report Host role, or to simply confirm enabling of Report Host role if it is a normal 
 *  connection and the attribute details are already known. 
 *
 ****************************************************************************************
 */
int app_hogprh_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct hogprh_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH enable confirmation status: 0x%X.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_hogprh_env[idx].conhdl = param->conhdl;
        app_hogprh_env[idx].enabled = true;
        app_hogprh_env[idx].hids_nb = param->hids_nb;

        for (uint8_t i = 0; i < param->hids_nb; i++)
        {
            // Set device to Report protocol mode
            app_hogprh_set_report_proto_mode_req(i, param->conhdl);
        }
        app_hogprh_rd_char_req(HOGPRH_RD_HIDS_REPORT_MAP, 0, 0, param->conhdl);
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     HOGPRH_WR_CHAR_RSP 
 * @param[in] param     Pointer to the struct hogprh_char_req_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the status of the writing request that has been 
 *  sent. 
 *
 ****************************************************************************************
 */
int app_hogprh_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_char_req_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH write char response status: 0x%X.\r\n", param->status);
    switch (param->att_code)
    {
    case HOGPRH_RD_WR_HIDS_PROTOCOL_MODE:
        break;
    case HOGPRH_RD_WR_HIDS_REPORT:
        break;
    default:
        break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for send Read Protocol Mode value to APP. *//**
 *
 * @param[in] msgid     HOGPRH_PROTO_MODE_RD_RSP 
 * @param[in] param     Pointer to the struct hogprh_proto_mode_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read Protocol Mode Characteristic value. 
 *
 ****************************************************************************************
 */
int app_hogprh_proto_mode_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_proto_mode_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH protocol mode read response mode(%d): ", param->hids_nb);

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
 * @brief Handles the generic message for Send Read Report Map value to APP. *//**
 *
 * @param[in] msgid     HOGPRH_REPORT_MAP_RD_RSP 
 * @param[in] param     Pointer to the struct hogprh_report_map_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read Report Map Characteristic value. 
 *
 ****************************************************************************************
 */
int app_hogprh_report_map_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_map_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH report map read response(%d):\r\n", param->hids_nb);
    QTRACE(param->report_map, param->report_map_length, 0, 2);
    QPRINTF("\r\n");
    uint8_t idx = KE_IDX_GET(src_id);
    if (param->hids_nb+1 < app_hogprh_env[idx].hids_nb) {
        app_hogprh_rd_char_req(HOGPRH_RD_HIDS_REPORT_MAP, 0, param->hids_nb+1, param->conhdl);
    }
    else {
        app_hogprh_rd_char_req(HOGPRH_RD_HIDS_HID_INFO, 0, 0, param->conhdl);
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read Report Map External Report Ref value to APP. *//**
 *
 * @param[in] msgid     HOGPRH_REPORT_MAP_EXT_REP_REF_RD_RSP 
 * @param[in] param     Pointer to the struct hogprh_report_map_ext_rep_ref_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read External Report Reference Descriptor 
 *  value. 
 *
 ****************************************************************************************
 */
int app_hogprh_report_map_ext_rep_ref_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_map_ext_rep_ref_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH external report ref read response(%d): 0x%04X\r\n", 
        param->hids_nb,
        param->ext_report_ref);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read Report Report Reference value to APP. *//**
 *
 * @param[in] msgid     HOGPRH_REPORT_REP_REF_RD_RSP 
 * @param[in] param     Pointer to the struct hogprh_report_ref_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read Report Reference Descriptor value. 
 *
 ****************************************************************************************
 */
int app_hogprh_report_ref_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_ref_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH report(%d) ref read response(%d), ID: %d, Type: %d\r\n", 
        param->report_nb,
        param->hids_nb,
        param->report_ref.report_id,
        param->report_ref.report_type
        );
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read HID Information value to APP. *//**
 *
 * @param[in] msgid     HOGPRH_HID_INFO_RD_RSP 
 * @param[in] param     Pointer to the struct hogprh_hid_info_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read HID Information Characteristic value. 
 *
 ****************************************************************************************
 */
int app_hogprh_hid_info_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_hid_info_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH hid info read response(%d):\rbcdHID: 0x%04X, bCountryCode: 0x%02X, Flags: 0x%02X\r\n", 
        param->hids_nb,
        param->hid_info.bcdHID,
        param->hid_info.bCountryCode,
        param->hid_info.flags
        );

    uint8_t idx = KE_IDX_GET(src_id);
    if (param->hids_nb+1 < app_hogprh_env[idx].hids_nb) {
        app_hogprh_rd_char_req(HOGPRH_RD_HIDS_HID_INFO, 0, param->hids_nb+1, param->conhdl);
    }
    else {
        // start notification
        app_hogprh_cfg_ntf_req(0, PRF_CLI_START_NTF, 0, param->conhdl); /* 0 is our demo input report */
        app_hogprh_env[idx].cur_code = 0;
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for send read notification configuration value to APP. *//**
 *
 * @param[in] msgid     HOGPRH_NTF_CFG_RD_RSP 
 * @param[in] param     Pointer to the struct hogprh_cfg_ntf_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  The API is used to inform the application about the read Client Characteristic Configuration 
 *  Descriptor value. 
 *
 ****************************************************************************************
 */
int app_hogprh_cfg_ntf_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_cfg_ntf_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH read char response(%d).\r\n", param->hids_nb);
   
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message Boot Mouse Input Report value send to APP
 *        (after Read Request or Notification) . *//**
 *
 * @param[in] msgid     HOGPRH_REPORT_IND 
 * @param[in] param     Pointer to the struct hogprh_report_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used to inform the application about the read Client Characteristic Configuration 
 *  Descriptor value. 
 *
 *  The following table present all the possible values for the ind_type parameter: 
 *  - HOGPRH_IND_NTF (0x00): The Report Characteristic value has been received has a 
 *  notification and the value is complete. 
 *  - HOGPRH_IND_RD_RSP (0x01): The Report Characteristic value has been received has a read 
 *  response. 
 *  - HOGPRH_IND_INCOMPLETE_NTF (0x02): The Report Characteristic value has been received has a 
 *  notification and the value is not complete. See the note below. 
 *
 *  @note
 *
 *  Here is an extract of the BLE HIDS specification, 
 *  "Notification of characteristic values can contain at most [ATT_MTU-3] bytes of data by definition. Data beyond 
 *  [ATT_MTU-3] bytes long is not included in a notification, and must instead be read using the GATT Read Long 
 *  Characteristic Value sub-procedure. The possibility that data to be notified in a Report characteristic value could 
 *  change before the HID Host completed an outstanding Read Long Characteristic Value sub-procedure, and therefore be 
 *  lost, exists. For this reason it is strongly recommended that HID Devices support an ATT_MTU large enough to transfer 
 *  their largest possible Report characteristic value in a single transaction."
 *
 *  Thus when an indication in received with an indication type set to HOGPRH_IND_INCOMPLETE_NTF, the application 
 *  can begin to parse this incomplete Report value. Then it must wait for another indication whose the indication type 
 *  will be set to HOGPRH_IND_RD_RSP and which will contain the whole Report Characteristic value (the first indication 
 *  can be discarded if needed). 
 *
 ****************************************************************************************
 */
int app_hogprh_report_ind_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);
    switch (param->ind_type)
    {
    case HOGPRH_IND_RD_RSP:
        QPRINTF("HOGPRH Read HIDS Report(%d).\r\n", param->hids_nb);
        QTRACE(param->report, param->report_length, 0, 2);
        QPRINTF("\r\n");
        break;
    case HOGPRH_IND_NTF:
        QPRINTF("HOGPRH Notification(%d).\r\n", param->hids_nb);
        QTRACE(param->report, param->report_length, 0, 2);
        QPRINTF("\r\n");
        if ((!app_hogprh_env[idx].cur_code) && (param->hids_nb+1 < app_hogprh_env[idx].hids_nb)) {
            // start notification
            app_hogprh_cfg_ntf_req(0, PRF_CLI_START_NTF, param->hids_nb+1, param->conhdl); /* 0 is our demo input report */
            app_hogprh_env[idx].cur_code = 1;
        }
        break;
    default:
        break;
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles read request response message from the HOGPRH. *//**
 *
 * @param[in] msgid     HOGPRH_RD_CHAR_ERR_RSP 
 * @param[in] param     Pointer to the struct hogprh_char_req_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 *
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is called when application sent a read request is not complient with the 
 *  specification or the implementation limitations.
 *
 ****************************************************************************************
 */
int app_hogprh_char_req_rsp_handler(ke_msg_id_t const msgid,
                                    struct hogprh_char_req_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH read char error response\r\n");
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     HOGPRH_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the HID Over GATT Profile Report Host 
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_hogprh_disable_ind_handler(ke_msg_id_t const msgid,
                                   struct prf_client_disable_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("HOGPRH disable ind\r\n");
    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_HOGPRH_TASK
