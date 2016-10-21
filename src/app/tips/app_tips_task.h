/**
 ****************************************************************************************
 *
 * @file app_tips_task.h
 *
 * @brief Application TIPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_TIPS_TASK_H_
#define APP_TIPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_TIPS_TASK Time Profile Profile Server Task API
 * @ingroup APP_TIPS
 * @brief Time Profile Profile Server Task API
 *
 * Time Profile Profile Server Task APIs are used to handle the message from TIPS or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_TIP_SERVER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_tips.h"

/// @cond

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// Time Profile Server environment variable
struct app_tips_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t features;
    // Connection handle
    uint16_t conhdl;
    // Current Time Notification flow control
    uint8_t ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_tips_env_tag *app_tips_env;

/// @endcond

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the TIPS.     
 *
 ****************************************************************************************
 */
int app_tips_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct tips_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the TIPS.    
 *
 ****************************************************************************************
 */
int app_tips_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct tips_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the eror indication message from the TIPS.    
 *
 ****************************************************************************************
 */
int app_tips_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the current time CCC indication message from the TIPS.   
 *
 ****************************************************************************************
 */
int app_tips_current_time_ccc_ind_handler(ke_msg_id_t const msgid,
                                          struct tips_current_time_ccc_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id);
/*
 ****************************************************************************************
 * @brief Handles the time update control indication message from the TIPS. 
 *
 ****************************************************************************************
 */
int app_tips_time_upd_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                          struct tips_time_upd_ctnl_pt_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id);
                             
#endif // BLE_TIP_SERVER

/// @} APP_TIPS_TASK

#endif // APP_TIPS_TASK_H_
