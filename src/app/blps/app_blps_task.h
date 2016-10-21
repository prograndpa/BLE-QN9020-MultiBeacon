/**
 ****************************************************************************************
 *
 * @file app_blps_task.h
 *
 * @brief Application BLPS task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BLPS_TASK_H_
#define APP_BLPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_BLPS_TASK Blood Pressure Profile Sensor Task API
 * @ingroup APP_BLPS
 * @brief Blood Pressure Profile Sensor Task API
 *
 * Blood Pressure Profile Sensor Task APIs are used to handle the message from BLPS or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_BP_SENSOR
#include "app_blps.h"

/// @cond

struct app_blps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t con_type;
    uint8_t features; // whether or not support cuff pressure measurement
    uint8_t evt_cfg;
    uint16_t conhdl;
    // Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_blps_env_tag *app_blps_env;

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles create database confirm message from the BLPS task   
 *
 ****************************************************************************************
 */
int app_blps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct blps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles disable service indication from the BLPS task 
 *
 ****************************************************************************************
 */
int app_blps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct blps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the configure ind/ntf indication from the BLPS task   
 *
 ****************************************************************************************
 */
int app_blps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct blps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the blood pressure measurement value confirm message from the BLPS 
 *
 ****************************************************************************************
 */
int app_blps_meas_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct blps_meas_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the BLPS task 
 *
 ****************************************************************************************
 */
int app_blps_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

#endif //BLE_BP_SENSOR

/// @} APP_BLPS_TASK

#endif // APP_BLPS_TASK_H_
