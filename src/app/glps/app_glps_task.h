/**
 ****************************************************************************************
 *
 * @file app_glps_task.h
 *
 * @brief Application GLPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_GLPS_TASK_H_
#define APP_GLPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_GLPS_TASK Glucose Profile Sensor Task API
 * @ingroup APP_GLPS
 * @brief Glucose Profile Sensor Task API
 *
 * Glucose Profile Sensor Task APIs are used to handle the message from GLPS or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_GL_SENSOR

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_glps.h"

/// @cond

// Glucose Profile Sensor environment variable
struct app_glps_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t evt_cfg;
    uint8_t records_idx;
    // Connection handle
    uint16_t conhdl;
    struct glp_racp_req racp_req;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_glps_env_tag *app_glps_env;

/*
 * TYPE DEFINITIONS
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
 * @brief Handles the create database confirmation from the GLPS.   
 *
 ****************************************************************************************
 */
int app_glps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct glps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the GLPS enable confirmation message from the GLPS.  
 *
 ****************************************************************************************
 */
int app_glps_enable_cfm_handler(ke_msg_id_t const msgid,
                                struct glps_enable_cfm *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the GLPS.    
 *
 ****************************************************************************************
 */
int app_glps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct glps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the GLPS.   
 *
 ****************************************************************************************
 */
int app_glps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct glps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the Record Access Control Point (RACP) request message from the GLPS. 
 *
 ****************************************************************************************
 */
int app_glps_racp_req_ind_handler(ke_msg_id_t const msgid,
                                  struct glps_racp_req_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);
                                
/*
 ****************************************************************************************
 * @brief Handles the request complete event message from the GLPS.    
 *
 ****************************************************************************************
 */
int app_glps_req_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct glps_req_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the GLPS.   
 *
 ****************************************************************************************
 */
int app_glps_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);
        
#endif // BLE_GL_SENSOR

/// @} APP_GLPS_TASK

#endif // APP_GLPS_TASK_H_
