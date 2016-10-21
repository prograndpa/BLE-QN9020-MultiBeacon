/**
 ****************************************************************************************
 *
 * @file app_hogprh_task.h
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
#ifndef APP_HOGPRH_TASK_H_
#define APP_HOGPRH_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HOGPRH_TASK HID Over GATT Profile Report Host Role TASK
 * @ingroup APP_HOGPRH
 * @brief HID Over GATT Profile Report Host Role TASK
 *
 * HID Over GATT Profile Report Host Role TASK APIs are used to handle the message from HOGPRH or APP.
 * 
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HID_REPORT_HOST
#include "app_hogprh.h"

/// @cond

/// HID Over GATT Profile Boot Host environment variable
struct app_hogprh_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
    uint8_t hids_nb;
    uint8_t cur_code;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_hogprh_env_tag *app_hogprh_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HOGPRH.
 *
 ****************************************************************************************
 */
int app_hogprh_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct hogprh_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_hogprh_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_char_req_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for send Read Protocol Mode value to APP
 *
 ****************************************************************************************
 */
int app_hogprh_proto_mode_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_proto_mode_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read Report Map value to APP
 *
 ****************************************************************************************
 */
int app_hogprh_report_map_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_map_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read Report Map External Report Ref value to APP
 *
 ****************************************************************************************
 */
int app_hogprh_report_map_ext_rep_ref_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_map_ext_rep_ref_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read Report Report Reference value to APP
 *
 ****************************************************************************************
 */
int app_hogprh_report_ref_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_ref_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for Send Read HID Information value to APP
 *
 ****************************************************************************************
 */
int app_hogprh_hid_info_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_hid_info_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for send read notification configuration value to APP
 *
 ****************************************************************************************
 */
int app_hogprh_cfg_ntf_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogprh_cfg_ntf_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message Boot Mouse Input Report value send to APP
 *        (after Read Request or Notification)
 *
 ****************************************************************************************
 */
int app_hogprh_report_ind_handler(ke_msg_id_t const msgid,
                      struct hogprh_report_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles read request response message from the HOGPRH.
 *
 ****************************************************************************************
 */
int app_hogprh_char_req_rsp_handler(ke_msg_id_t const msgid,
                                    struct hogprh_char_req_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
                                    
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_hogprh_disable_ind_handler(ke_msg_id_t const msgid,
                                   struct prf_client_disable_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

#endif // BLE_HID_REPORT_HOST

/// @} APP_HOGPRH_TASK

#endif // APP_HOGPRH_TASK_H_
