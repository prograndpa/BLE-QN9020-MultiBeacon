/**
 ****************************************************************************************
 *
 * @file app_anpc_task.h
 *
 * @brief Application ANPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_ANPC_TASK_H_
#define APP_ANPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_ANPC_TASK Alert Notification Client Role Task API
 * @ingroup APP_ANPC
 * @brief Alert Notification Client Role Task API
 *
 * Alert Notification Client Role Task APIs are used to handle the message from ANPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_AN_CLIENT
#include "app_anpc.h"
/// @cond

/// Alert Notification Client environment variable
struct app_anpc_env_tag
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
extern struct app_anpc_env_tag *app_anpc_env;
/// @endcond
/*
 ****************************************************************************************
 * @brief Indicate to APP that the ANPC value has been received.
 *
 ****************************************************************************************
 */
int app_anpc_value_ind_handler(ke_msg_id_t const msgid,
                               struct anpc_value_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Indicate the content of the peer device Alert Notification service.
 *
 ****************************************************************************************
 */
int app_anpc_content_ind_handler(ke_msg_id_t const msgid,
                                 struct anpc_ans_content_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Complete Event Information.
 *
 ****************************************************************************************
 */
int app_anpc_cmp_evt_handler(ke_msg_id_t const msgid,
                             struct anpc_cmp_evt *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_anpc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

#endif // BLE_AN_CLIENT

/// @} APP_ANPC_TASK

#endif // APP_ANPC_TASK_H_
