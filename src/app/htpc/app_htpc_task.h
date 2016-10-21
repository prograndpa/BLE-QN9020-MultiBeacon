/**
 ****************************************************************************************
 *
 * @file app_htpc_task.h
 *
 * @brief Application HTPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_HTPC_TASK_H_
#define APP_HTPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HTPC_TASK Health Thermometer Profile Collector Task API
 * @ingroup APP_HTPC
 * @brief Health Thermometer Profile Collector Task API
 *
 * Health Thermometer Profile Collector Task APIs are used to handle the message from HTPC or APP.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HT_COLLECTOR
#include "app_htpc.h"
/// @cond

/// Health Thermometer Profile Client environment variable
struct app_htpc_env_tag
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
extern struct app_htpc_env_tag *app_htpc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HTPC.
 ****************************************************************************************
 */
int app_htpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct htpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the HTPC.
 ****************************************************************************************
 */
int app_htpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct htpc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);


/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 ****************************************************************************************
 */
int app_htpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct htpc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 ****************************************************************************************
 */
int app_htpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct htpc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);


/*
 ****************************************************************************************
 * @brief Handles the temperature value send to APP
 ****************************************************************************************
 */
int app_htpc_temp_ind_handler(ke_msg_id_t const msgid,
                      struct htpc_temp_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the measurement interval value indication to APP
 ****************************************************************************************
 */
int app_htpc_meas_intv_ind_handler(ke_msg_id_t const msgid,
                      struct htpc_meas_intv_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP
 ****************************************************************************************
 */
int app_htpc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

#endif // BLE_HT_COLLECTOR

/// @} APP_HTPC_TASK

#endif // APP_HTPC_TASK_H_
