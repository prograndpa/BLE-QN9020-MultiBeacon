/**
 ****************************************************************************************
 *
 * @file app_diss_task.h
 *
 * @brief Application DISS task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_DISS_TASK_H_
#define APP_DISS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_DISS_TASK Device Information Service Server Task API
 * @ingroup APP_DISS
 * @brief Device Information Service Server Task API
 *
 * Device Information Service Server Task APIs are used to handle the message from DISS or APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_DIS_SERVER
#include "app_diss.h"

/// @cond
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
struct app_diss_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t status;	
    uint8_t con_type;
    uint16_t features;
    uint16_t conhdl;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_diss_env_tag *app_diss_env;

/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles create database message. 
 *
 ****************************************************************************************
 */
int app_diss_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct diss_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles error indication message.   
 *
 ****************************************************************************************
 */
 int app_diss_error_ind_handler(ke_msg_id_t const msgid,
                                struct prf_server_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

#endif // BLE_DIS_SERVER

/// @} APP_DISS_TASK

#endif // APP_DISS_TASK_H_
