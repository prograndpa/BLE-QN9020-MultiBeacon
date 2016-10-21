/**
 ****************************************************************************************
 *
 * @file app_pasps_task.h
 *
 * @brief Application PASPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_PASPS_TASK_H_
#define APP_PASPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_PASPS_TASK Phone Alert Status Profile Server Task API
 * @ingroup APP_PASPS
 * @brief Phone Alert Status Profile Server Task API
 *
 * Phone Alert Status Profile Server Task APIs are used to handle the message from PASPS or APP.
 * @{
 ****************************************************************************************
 */
 
#if BLE_PAS_SERVER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/// @cond

struct app_pasps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    // connection handle
    uint16_t conhdl;
    uint16_t app_cfg;
    // Notification flow control
    uint8_t ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_pasps_env_tag *app_pasps_env;
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
 * @brief Handles the disable service indication from the PASPS.
 *
 ****************************************************************************************
 */
int app_pasps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct pasps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the written characteristic indication from the PASPS.
 *
 ****************************************************************************************
 */
int app_pasps_written_char_val_ind_handler(ke_msg_id_t const msgid,
                                           struct pasps_written_char_val_ind *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the command complete event from the PASPS.
 *
 ****************************************************************************************
 */
int app_pasps_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct pasps_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

#endif // BLE_PAS_SERVER

/// @} APP_PASPS_TASK

#endif // APP_PASPS_TASK_H_
