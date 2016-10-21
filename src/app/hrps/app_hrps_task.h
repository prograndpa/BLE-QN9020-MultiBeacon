/**
 ****************************************************************************************
 *
 * @file app_hrps_task.h
 *
 * @brief Application HRPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_HRPS_TASK_H_
#define APP_HRPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HRPS_TASK Heart Rate Profile Sensor Task API
 * @ingroup APP_HRPS
 * @brief Heart Rate Profile Sensor Task API
 *
 * Heart Rate Profile Sensor Task APIs are used to handle the message from HRPS or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_HR_SENSOR

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_hrps.h"

/// @cond

// Heart Rate Profile Server environment variable
struct app_hrps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t features;
    // Connection handle
    uint16_t conhdl;
    uint16_t energy_expended;
    // Current Time Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_hrps_env_tag *app_hrps_env;

/// @endcond
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

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
 * @brief Handles the create database confirmation from the HRPS.   
 *
 ****************************************************************************************
 */
int app_hrps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct hrps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the HRPS. 
 *
 ****************************************************************************************
 */
int app_hrps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct hrps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the eror indication nessage from the HRPS.  
 *
 ****************************************************************************************
 */
int app_hrps_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the send means confirm message from the HRPS.  
 *
 ****************************************************************************************
 */
int app_hrps_means_send_cfm_handler(ke_msg_id_t const msgid,
                                    struct hrps_meas_send_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the HRPS. 
 *
 ****************************************************************************************
 */
int app_hrps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct hrps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the energy exp reset ind message from the HRPS.  
 *
 ****************************************************************************************
 */
int app_hrps_energy_exp_reset_ind_handler(ke_msg_id_t const msgid,
                                          struct hrps_energy_exp_reset_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id);

#endif // BLE_HR_SENSOR

/// @} APP_HRPS_TASK

#endif // APP_HRPS_TASK_H_
