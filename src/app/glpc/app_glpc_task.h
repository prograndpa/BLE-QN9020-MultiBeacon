/**
 ****************************************************************************************
 *
 * @file app_glpc_task.h
 *
 * @brief Application GLPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_GLPC_TASK_H_
#define APP_GLPC_TASK_H_


/**
 ****************************************************************************************
 * @addtogroup APP_GLPC_TASK Glucose Profile Collector Task API
 * @ingroup APP_GLPC
 * @brief Glucose Profile Collector Task API
 *
 * Glucose Profile Collector Task APIs are used to handle the message from GLPC or APP.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_GL_COLLECTOR
#include "app_glpc.h"

/// @cond
// Current Operation State
enum
{
    OPERATION_STATE_IDLE,
    OPERATION_STATE_ONGOING,
};

/// Blood Pressure Profile Client environment variable
struct app_glpc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
    /// Glucose sensor features
    uint16_t features;

    uint8_t op_state;
    /// counter used to check service uniqueness
    uint8_t nb_svc;
    uint8_t cur_code;    
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_glpc_env_tag *app_glpc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the GLPC.
 *
 ****************************************************************************************
 */
int app_glpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct glpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the register to Glucose sensor notifications - confirmation.
 *
 ****************************************************************************************
 */
int app_glpc_register_cfm_handler(ke_msg_id_t const msgid,
                      struct glpc_register_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read Glucose sensor features - response
 *
 ****************************************************************************************
 */
int app_glpc_read_features_rsp_handler(ke_msg_id_t const msgid,
                      struct glpc_read_features_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message of record Access Control Point - Resp
 *
 ****************************************************************************************
 */
int app_glpc_racp_response_handler(ke_msg_id_t const msgid,
                      struct glpc_racp_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for Glucose measurement value received
 *
 ****************************************************************************************
 */
int app_glpc_meas_ind_handler(ke_msg_id_t const msgid,
                      struct glpc_meas_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for Glucose measurement context received
 *
 ****************************************************************************************
 */
int app_glpc_meas_ctx_ind_handler(ke_msg_id_t const msgid,
                      struct glpc_meas_ctx_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_glpc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

#endif // BLE_GL_COLLECTOR

/// @} APP_GLPC_TASK

#endif // APP_GLPC_TASK_H_
