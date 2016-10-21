/**
 ****************************************************************************************
 *
 * @file app_cscps_task.h
 *
 * @brief Application CSCPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_CSCPS_TASK_H_
#define APP_CSCPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_CSCPS_TASK Cycling Speed and Cadence Profile Sensor Task API
 * @ingroup APP_CSCPS
 * @brief Cycling Speed and Cadence Profile Sensor Task API
 *
 * Cycling Speed and Cadence Profile Sensor Task APIs are used to handle the message from CSCPS or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_CSC_SENSOR
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/// @cond

struct app_cscps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    // connection handle
    uint16_t conhdl;
    uint16_t app_cfg;
    uint32_t wheel_revol;
    // Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_cscps_env_tag *app_cscps_env;

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
 * @brief Handles the disable service indication from the CSCPS.
 *
 ****************************************************************************************
 */
int app_cscps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct cscps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the control point indication from the CSCPS.
 *
 ****************************************************************************************
 */
int app_cscps_ctnl_pt_req_ind_handler(ke_msg_id_t const msgid,
                                      struct cscps_sc_ctnl_pt_req_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the CSCPS.
 *
 ****************************************************************************************
 */
int app_cscps_ntf_ind_cfg_ind_handler(ke_msg_id_t const msgid,
                                      struct cscps_ntf_ind_cfg_ind *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the command complete event from the CSCPS.
 *
 ****************************************************************************************
 */
int app_cscps_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct cscps_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

#endif // BLE_CSC_SENSOR

/// @} APP_CSCPS_TASK

#endif // APP_CSCPS_TASK_H_
