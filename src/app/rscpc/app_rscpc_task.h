/**
 ****************************************************************************************
 *
 * @file app_rscpc_task.h
 *
 * @brief Application RSCPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_RPCPC_TASK_H_
#define APP_RPCPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_RSCPC_TASK Running Speed and Cadence Profile Collector Task API
 * @ingroup APP_RSCPC
 * @brief Running Speed and Cadence Profile Collector Task API
 *
 * Running Speed and Cadence Profile Collector Task APIs are used to handle the message from RSCPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_RSC_COLLECTOR
#include "app_rscpc.h"

/// @cond

/// Running Speed and Cadence Profile Collector environment variable
struct app_rscpc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_rscpc_env_tag *app_rscpc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the result of discovery procedure from TASK_RSCPC.
 *
 ****************************************************************************************
 */
int app_rscpc_rscs_content_ind_handler(ke_msg_id_t const msgid,
                                       struct rscpc_rscs_content_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the value indication from TASK_RSCPC.
 *
 ****************************************************************************************
 */
int app_rscpc_value_ind_handler(ke_msg_id_t const msgid,
                                struct rscpc_value_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the complete event from TASK_RSCPC.
 *
 ****************************************************************************************
 */
int app_rscpc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct rscpc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_rscpc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

#endif // BLE_RSC_COLLECTOR

/// @} APP_RSCPC_TASK

#endif // APP_RPCPC_TASK_H_
