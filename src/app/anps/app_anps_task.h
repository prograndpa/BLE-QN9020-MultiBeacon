/**
 ****************************************************************************************
 *
 * @file app_anps_task.h
 *
 * @brief Application ANPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_ANPS_TASK_H_
#define APP_ANPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_ANPS_TASK Alert Notification Server Task API
 * @ingroup APP_ANPS
 * @brief Alert Notification Server Task API
 *
 * Alert Notification Server Task APIs are used to handle the message from TASK_ANPS or APP.
 *
 * @{
 ****************************************************************************************
 */

#if BLE_AN_SERVER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_anps.h"

/// @cond

/// Alert Notification Server environment variable
struct app_anps_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Supported New Alert Category Characteristic Value
    uint16_t supp_new_alert_cat;
    /// Supported Unread Alert Category Characteristic Value
    uint16_t supp_unread_alert_cat;
    /// Connection handle
    uint16_t conhdl;

    /// Client Characteristic Configuration Status
    uint8_t ntf_cfg;
    /// New Alert Characteristic - Saved Client Characteristic Configuration Descriptor Value
    uint16_t alert_category_id;
    /// Unread Alert Status Characteristic - Saved Client Characteristic Configuration Descriptor Value
    uint16_t unread_alert_category_id;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_anps_env_tag *app_anps_env;

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @endcond

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the ANPS.
 *
 ****************************************************************************************
 */
int app_anps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct anps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles requests to be notified about new alert values.
 *
 ****************************************************************************************
 */

int app_anps_ntf_immediate_req_ind_handler(ke_msg_id_t const msgid,
                                           struct anps_ntf_immediate_req_ind *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles Indicate that the notification configuration has been modified by the 
 *        peer device.
 *
 ****************************************************************************************
 */
int app_anps_ntf_status_update_ind_handler(ke_msg_id_t const msgid,
                                           struct anps_ntf_status_update_ind *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles Complete Event Information .
 *
 ****************************************************************************************
 */
int app_anps_cmp_evt_handler(ke_msg_id_t const msgid,
                             struct anps_cmp_evt *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);
#endif // BLE_AN_SERVER

/// @} APP_ANPS_TASK

#endif // APP_ANPS_TASK_H_
