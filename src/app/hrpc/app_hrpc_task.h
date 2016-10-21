/**
 ****************************************************************************************
 *
 * @file app_hrpc_task.h
 *
 * @brief Application HRPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_HRPC_TASK_H_
#define APP_HRPC_TASK_H_


/**
 ****************************************************************************************
 * @addtogroup APP_HRPC_TASK Heart Rate Profile Collector Task API
 * @ingroup APP_HRPC
 * @brief Heart Rate Profile Collector Task API
 *
 * Heart Rate Profile Collector Task APIs are used to handle the message from HRPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HR_COLLECTOR
#include "app_hrpc.h"
/// @cond

/// Heart Rate Profile Colletor environment variable
struct app_hrpc_env_tag
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
extern struct app_hrpc_env_tag *app_hrpc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HRPC.
 *
 ****************************************************************************************
 */
int app_hrpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct hrpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the HRPC.
 *
 ****************************************************************************************
 */
int app_hrpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct hrpc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 *
 ****************************************************************************************
 */
int app_hrpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hrpc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_hrpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hrpc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Heart Rate value send to APP
 *
 ****************************************************************************************
 */
int app_hrpc_meas_ind_handler(ke_msg_id_t const msgid,
                      struct hrpc_meas_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP
 *
 ****************************************************************************************
 */
int app_hrpc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

#endif // BLE_HR_COLLECTOR

/// @} APP_HRPC_TASK

#endif // APP_HRPC_TASK_H_
