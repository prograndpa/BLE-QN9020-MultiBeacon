/**
 ****************************************************************************************
 *
 * @file app_bass_task.h
 *
 * @brief Application BASS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BASS_TASK_H_
#define APP_BASS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_BASS_TASK Battery Service Server Task API
 * @ingroup APP_BASS
 * @brief Battery Service Server Task API
 *
 * Battery Service Service Task APIs are used to handle the message from TASK_BASS or APP.
 *
 * @{
 ****************************************************************************************
 */

#if BLE_BATT_SERVER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_bass.h"

/// @cond

// Battery Service Server environment variable
struct app_bass_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t bas_nb;
    uint8_t features[BASS_NB_BAS_INSTANCES_MAX];
    // Connection handle
    uint16_t conhdl;
    
    // Battery Level Notification Configurations
    uint16_t batt_level_ntf_cfg[BASS_NB_BAS_INSTANCES_MAX];
    // Last Battery Level
    uint8_t old_batt_lvl[BASS_NB_BAS_INSTANCES_MAX];
    // Current Battery Level
    uint8_t current_batt_lvl[BASS_NB_BAS_INSTANCES_MAX];
    // Battery Level Characteristic Presentation Format - Should not change during connection
    struct prf_char_pres_fmt batt_level_pres_format[BASS_NB_BAS_INSTANCES_MAX];
    // Current Time Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_bass_env_tag *app_bass_env;

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
 * @brief Handles the create database confirmation from the BASS.  
 *
 ****************************************************************************************
 */
int app_bass_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct bass_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the disable database confirmation from the BASS. 
 *
 ****************************************************************************************
 */
int app_bass_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct bass_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);
                             
/*
 ****************************************************************************************
 * @brief Handles the error indication from the BASS.  
 *
 ****************************************************************************************
 */
int app_bass_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);
                           

/*
 ****************************************************************************************
 * @brief Handles the battery level update confirm message from the BASS.      
 *
 ****************************************************************************************
 */
int app_bass_batt_level_upd_cfm_handler(ke_msg_id_t const msgid,
                                        struct bass_batt_level_upd_cfm *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);
                                    
/*
 ****************************************************************************************
 * @brief Handles the battery level update confirm message from the BASS.      
 *
 ****************************************************************************************
 */
int app_bass_batt_level_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                            struct bass_batt_level_ntf_cfg_ind *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id);

#endif // BLE_BATT_SERVER

/// @} APP_BASS_TASK

#endif // APP_BASS_TASK_H_
