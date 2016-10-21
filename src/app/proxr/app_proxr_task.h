/**
 ****************************************************************************************
 *
 * @file app_proxr_task.h
 *
 * @brief Application PROXR implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_PROXR_TASK_H_
#define APP_PROXR_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_PROXR_TASK Proximity profile Reporter Task API
 * @ingroup APP_PROXR
 * @brief Proximity profile Reporter Task API
 *
 * Proximity profile Reporter Task APIs are used to handle the message from PROXR or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_PROX_REPORTER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_proxr.h"

/// @cond

// Proximity Profile Reporter environment variable
struct app_proxr_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t features;
    uint8_t link_loss_lvl;
    uint8_t alert_lvl;
    // Connection handle
    uint16_t conhdl;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_proxr_env_tag *app_proxr_env;

/// @endcond

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
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
 * @brief Handles the create database confirmation from the PROXR.    
 *
 ****************************************************************************************
 */
int app_proxr_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct proxr_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the PROXR.
 *
 ****************************************************************************************
 */
int app_proxr_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct proxr_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);
                              
/*
 ****************************************************************************************
 * @brief Handles alert indication from the PROXR. 
 *
 ****************************************************************************************
 */ 
int app_proxr_alert_ind_handler(ke_msg_id_t const msgid,
                                struct proxr_alert_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

#endif // BLE_PROX_REPORTER

/// @} APP_PROXR_TASK

#endif // APP_PROXR_TASK_H_
