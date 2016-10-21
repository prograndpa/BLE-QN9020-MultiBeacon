/**
 ****************************************************************************************
 *
 * @file app_scpps_task.h
 *
 * @brief Application SCPPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_SCPPS_TASK_H_
#define APP_SCPPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_SCPPS_TASK Scan Paramters Profile Server Task API
 * @ingroup APP_SCPPS
 * @brief Scan Paramters Profile Server Task API
 *
 * Scan Paramters Profile Server Task APIs are used to handle the message from SCPPS or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_SP_SERVER

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_scpps.h"

/// @cond

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


// Scan Parameters Profile Server environment variable
struct app_scpps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t features;
    // Connection handle
    uint16_t conhdl;
    // Scan Refresh NTF Configuration
    uint8_t scan_refresh_ntf_cfg;
    struct scan_intv_wd scan_intv_wd;
    // Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_scpps_env_tag *app_scpps_env;

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
 * @brief Handles the create database confirmation from the SCPPS.     
 *
 ****************************************************************************************
 */
int app_scpps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct scpps_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the SCPPS.   
 *
 ****************************************************************************************
 */
int app_scpps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct scpps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the eror indication message from the SCPPS.     
 *
 ****************************************************************************************
 */
int app_scpps_error_ind_handler(ke_msg_id_t const msgid,
                                struct prf_server_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the scan interval indication message from the SCPPS.  
 *
 ****************************************************************************************
 */
int app_scpps_scan_intv_wd_ind_handler(ke_msg_id_t const msgid,
                                       struct scpps_scan_intv_wd_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the scan refresh notification configuration message from the SCPPS. 
 *
 ****************************************************************************************
 */
int app_scpps_scan_refresh_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                               struct scpps_scan_refresh_ntf_cfg_ind *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the scan refresh confirm message from the SCPPS.    
 *
 ****************************************************************************************
 */
int app_scpps_scan_refresh_send_cfm_handler(ke_msg_id_t const msgid,
                                            struct scpps_scan_refresh_send_cfm *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id);

#endif // BLE_SP_SERVER

/// @} APP_SCPPS_TASK

#endif // APP_SCPPS_TASK_H_
