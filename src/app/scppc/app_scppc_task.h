/**
 ****************************************************************************************
 *
 * @file app_scppc_task.h
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

#ifndef APP_SCPPC_TASK_H_
#define APP_SCPPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_SCPPC_TASK Scan Parameters Profile Client Task API
 * @ingroup APP_SCPPC
 * @brief Scan Parameters Profile Client Task API
 *
 * Scan Parameters Profile Client Task APIs are used to handle the message from SCPPC or APP. 
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_SP_CLIENT
#include "app_scppc.h"
/// @cond

/// Scan Parameter Profile Client environment variable
struct app_scppc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
    uint8_t cur_code;    
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_scppc_env_tag *app_scppc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the SCPPC.
 *
 ****************************************************************************************
 */
int app_scppc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct scppc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the SCPPC.
 *
 ****************************************************************************************
 */
int app_scppc_error_ind_handler(ke_msg_id_t const msgid,
                      struct scppc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_scppc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct scppc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Message of Scan Refresh Notification Configuration has been read
 *
 ****************************************************************************************
 */
int app_scppc_scan_refresh_ntf_cfg_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct scppc_scan_refresh_ntf_cfg_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP
 *
 ****************************************************************************************
 */
int app_scppc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

#endif // BLE_SP_CLIENT

/// @} APP_SCPPC_TASK

#endif // APP_SCPPC_TASK_H_
