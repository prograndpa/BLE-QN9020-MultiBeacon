/**
 ****************************************************************************************
 *
 * @file app_paspc_task.h
 *
 * @brief Application PASPC implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_PASPC_TASK_H_
#define APP_PASPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_PASPC_TASK Phone Alert Status Profile Client Task API
 * @ingroup APP_PASPC
 * @brief Phone Alert Status Profile Client Task API
 *
 * Phone Alert Status Profile Client Task APIs are used to handle the message from PASPC or APP.
 * @{
 ****************************************************************************************
 */
 
#if BLE_PAS_CLIENT

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/// @cond

/// Phone Alert Status Profile Client environment variable
struct app_paspc_env_tag
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
extern struct app_paspc_env_tag *app_paspc_env;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_paspc_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Indicate that an attribute value has been received either upon notification
 *        or read response.
 *
 ****************************************************************************************
 */
int app_paspc_val_ind_handler(ke_msg_id_t const msgid,
                              struct paspc_value_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Indicate the content of the pointer device Phone Alert Status service.
 *
 ****************************************************************************************
 */
int app_paspc_pass_content_handler(ke_msg_id_t const msgid,
                                   struct paspc_pass_content_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the complete event from TASK_PASPC.
 *
 ****************************************************************************************
 */
int app_paspc_cmp_evt_handler(ke_msg_id_t const msgid,
                              struct paspc_cmp_evt *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);
#endif // BLE_PAS_CLIENT

/// @} APP_PASPC_TASK

#endif // APP_PASPC_TASK_H_
