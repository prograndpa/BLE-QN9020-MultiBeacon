/**
 ****************************************************************************************
 *
 * @file app_qpps_task.h
 *
 * @brief Application QPPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */
 
#ifndef APP_QPPS_TASK_H_
#define APP_QPPS_TASK_H_

#if BLE_QPP_SERVER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_qpps.h"

/// @cond

//Quintic Private Profile Server environment variable
struct app_qpps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t tx_char_num;
    #if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)
    uint8_t tx_buffer_available;
    #endif
    // Connection handle
    uint16_t conhdl;
    uint32_t features;
    uint32_t char_status;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_qpps_env_tag *app_qpps_env;

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

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
 * @brief Handles the create database confirmation from the QPPS.   
 *
 ****************************************************************************************
 */
int app_qpps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct qpps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable database confirmation from the QPPS. 
 *
 ****************************************************************************************
 */
int app_qpps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct qpps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the error indication nessage from the QPPS.  
 *
 ****************************************************************************************
 */
int app_qpps_error_ind_handler(ke_msg_id_t const msgid,
                               struct qpps_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the send data confirm message from the QPPS.  
 *
 ****************************************************************************************
 */
int app_qpps_data_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct qpps_data_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the QPPS. 
 *
 ****************************************************************************************
 */
int app_qpps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct qpps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the data indication message from the QPPS.  
 *
 ****************************************************************************************
 */
int app_qpps_data_ind_handler(ke_msg_id_t const msgid,
                              struct qpps_data_val_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

#endif // BLE_QPP_SERVER

#endif // APP_QPPS_TASK_H_
