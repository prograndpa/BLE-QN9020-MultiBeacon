/**
 ****************************************************************************************
 *
 * @file app_rscps_task.h
 *
 * @brief Application RSCPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_RSCPS_TASK_H_
#define APP_RSCPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_RSCPS_TASK Running Speed and Cadence Profile Sensor Task API
 * @ingroup APP_RSCPS
 * @brief Running Speed and Cadence Profile Sensor Task API
 *
 * Running Speed and Cadence Profile Sensor Task APIs are used to handle the message from RSCPS or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_RSC_SENSOR

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/// @cond

struct app_rscps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    // connection handle
    uint16_t conhdl;
    uint16_t app_cfg;
    // Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_rscps_env_tag *app_rscps_env;
/// @endcond

/*
 ****************************************************************************************
 * @brief 
 *
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the RSCPS.
 *
 ****************************************************************************************
 */
int app_rscps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct rscps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the control point indication from the RSCPS.
 *
 ****************************************************************************************
 */
int app_rscps_ctnl_pt_req_ind_handler(ke_msg_id_t const msgid,
                                      struct rscps_sc_ctnl_pt_req_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the RSCPS.
 *
 ****************************************************************************************
 */
int app_rscps_ntf_ind_cfg_ind_handler(ke_msg_id_t const msgid,
                                      struct rscps_ntf_ind_cfg_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the command complete event from the RSCPS.
 *
 ****************************************************************************************
 */
int app_rscps_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct rscps_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

#endif // BLE_RSC_SENSOR

/// @} APP_RSCPS_TASK

#endif // APP_RSCPS_TASK_H_
