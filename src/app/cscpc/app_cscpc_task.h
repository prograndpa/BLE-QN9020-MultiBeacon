/**
 ****************************************************************************************
 *
 * @file app_cscpc_task.h
 *
 * @brief Application CSCPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_CPCPC_TASK_H_
#define APP_CPCPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_CSCPC_TASK Cycling Speed and Cadence Profile Collector Task API
 * @ingroup APP_CSCPC
 * @brief Cycling Speed and Cadence Profile Collector Task API
 *
 * Cycling Speed and Cadence Profile Collector Task APIs are used to handle the message from CSCPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_CSC_COLLECTOR
#include "app_cscpc.h"
/// @cond

/// Cycling Speed and Cadence Profile Collector environment variable
struct app_cscpc_env_tag
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
extern struct app_cscpc_env_tag *app_cscpc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable result from TASK_CSCPC.
 *
 ****************************************************************************************
 */
int app_cscpc_cscs_content_ind_handler(ke_msg_id_t const msgid,
                                       struct cscpc_cscs_content_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);
                                       
/*
 ****************************************************************************************
 * @brief Handles the value indication from TASK_CSCPC.
 *
 ****************************************************************************************
 */
int app_cscpc_value_ind_handler(ke_msg_id_t const msgid,
                                struct cscpc_value_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the complete event from TASK_CSCPC.
 *
 ****************************************************************************************
 */
int app_cscpc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct cscpc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_cscpc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

#endif // BLE_CSC_COLLECTOR

/// @} APP_CSCPC_TASK

#endif // APP_CPCPC_TASK_H_

