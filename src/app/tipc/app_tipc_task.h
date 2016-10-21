/**
 ****************************************************************************************
 *
 * @file app_tipc_task.h
 *
 * @brief Application TIPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_TIPC_TASK_H_
#define APP_TIPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_TIPC_TASK Time Profile Client Task API
 * @ingroup APP_TIPC
 * @brief Time Profile Client Task API
 *
 * Time Profile Client Task APIs are used to handle the message from TIPC or APP. 
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_TIP_CLIENT
#include "app_tipc.h"
/// @cond

/// Time Profile Client environment variable
struct app_tipc_env_tag
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
extern struct app_tipc_env_tag *app_tipc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the TIPC.
 *
 ****************************************************************************************
 */
int app_tipc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct tipc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the TIPC.
 *
 ****************************************************************************************
 */
int app_tipc_error_ind_handler(ke_msg_id_t const msgid,
                      struct tipc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Generic message for writing characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_tipc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Received Current Time value sent to APP (Read Response or Notification)
 *
 ****************************************************************************************
 */
int app_tipc_ct_ind_handler(ke_msg_id_t const msgid,
                      struct tipc_ct_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Read Current Time Notification Configuration sent to APP
 *
 ****************************************************************************************
 */
int app_tipc_ct_ntf_cfg_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_ct_ntf_cfg_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Read Local Time Info value sent to APP
 *
 ****************************************************************************************
 */
int app_tipc_lti_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_lti_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Read Reference Time Info value sent to APP
 *
 ****************************************************************************************
 */
int app_tipc_rti_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_rti_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Read Time With DST value sent to APP
 *
 ****************************************************************************************
 */
int app_tipc_tdst_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_tdst_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Read Time Update State value sent to APP
 *
 ****************************************************************************************
 */
int app_tipc_tus_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_tus_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_tipc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

#endif // BLE_TIP_CLIENT

/// @} APP_TIPC_TASK

#endif // APP_TIPC_TASK_H_
