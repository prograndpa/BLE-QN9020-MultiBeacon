/**
 ****************************************************************************************
 *
 * @file app_hogpbh_task.h
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
#ifndef APP_HOGPBH_TASK_H_
#define APP_HOGPBH_TASK_H_


/**
 ****************************************************************************************
 * @addtogroup APP_HOGPBH_TASK HID Over GATT Profile Boot Host Role TASK
 * @ingroup APP_HOGPBH
 * @brief HID Over GATT Profile Boot Host Role TASK
 *
 * HID Over GATT Profile Boot Host Role TASK APIs are used to handle the message from HOGPBH or APP.
 * 
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HID_BOOT_HOST
#include "app_hogpbh.h"

/// @cond

/// HID Over GATT Profile Boot Host environment variable
struct app_hogpbh_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
    /// Instance Number of Mouse
    uint8_t hids_mouse;
    /// Instance Number of Keyboard
    uint8_t hids_kb;
    uint8_t cur_code;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_hogpbh_env_tag *app_hogpbh_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HOGPBH.
 *
 ****************************************************************************************
 */
int app_hogpbh_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct hogpbh_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_hogpbh_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hogpbh_char_req_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for send read notification configuration value to APP
 *
 ****************************************************************************************
 */
int app_hogpbh_cfg_ntf_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogpbh_cfg_ntf_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for send Read Protocol Mode value to APP
 *
 ****************************************************************************************
 */
int app_hogpbh_proto_mode_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct hogpbh_proto_mode_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message Boot Report value send to APP. *//**
 *

 ****************************************************************************************
 */
int app_hogpbh_boot_report_ind_handler(ke_msg_id_t const msgid,
                                       struct hogpbh_boot_report_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles read request response message from the HOGPBH.
 *
 ****************************************************************************************
 */
int app_hogpbh_char_req_rsp_handler(ke_msg_id_t const msgid,
                                    struct hogpbh_char_req_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_hogpbh_disable_ind_handler(ke_msg_id_t const msgid,
                                   struct prf_client_disable_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

#endif // BLE_HID_BOOT_HOST

/// @} APP_HOGPBH_TASK

#endif // APP_HOGPBH_TASK_H_
