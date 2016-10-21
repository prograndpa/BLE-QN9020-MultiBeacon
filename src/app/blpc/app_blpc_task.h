/**
 ****************************************************************************************
 *
 * @file app_blpc_task.h
 *
 * @brief Application BLPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_BLPC_TASK_H_
#define APP_BLPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_BLPC_TASK Blood Pressure Profile Collector Task API
 * @ingroup APP_BLPC
 * @brief Blood Pressure Profile Collector Task API
 *
 * Blood Pressure Profile Collector Task APIs are used to handle the message from BLPC or APP.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_BP_COLLECTOR
#include "app_blpc.h"
/// @cond

/// Blood Pressure Profile Client environment variable
struct app_blpc_env_tag
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
extern struct app_blpc_env_tag *app_blpc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the BLPC.
 *
 ****************************************************************************************
 */
int app_blpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct blpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the BLPC.
 *
 ****************************************************************************************
 */
int app_blpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct blpc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 *
 ****************************************************************************************
 */
int app_blpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct blpc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_blpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct blpc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Blood pressure value send to APP
 *
 ****************************************************************************************
 */
int app_blpc_bp_meas_ind_handler(ke_msg_id_t const msgid,
                      struct blpc_meas_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_blpc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

#endif // BLE_BP_COLLECTOR

/// @} APP_BLPC_TASK

#endif // APP_BLPC_TASK_H_
