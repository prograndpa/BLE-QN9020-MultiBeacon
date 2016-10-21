/**
 ****************************************************************************************
 *
 * @file app_findt_task.h
 *
 * @brief Application FINDT task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_FINDT_TASK_H_
#define APP_FINDT_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_FINDT_TASK Find Me Profile Target Task API
 * @ingroup APP_FINDT
 * @brief Find Me Profile Target Task API
 *
 * Find Me Profile Target Task APIs are used to handle the message from FINDT or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_FINDME_TARGET
#include "app_findt.h"

/// @cond

struct app_findt_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t alert_lvl;
    // Connection handle
    uint16_t conhdl;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_findt_env_tag *app_findt_env;

/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the create DB confirmation from the FINDT. 
 *
 ****************************************************************************************
 */
int app_findt_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct findt_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the alert indication from the FINDT. 
 *
 ****************************************************************************************
 */
int app_findt_alert_ind_handler(ke_msg_id_t const msgid,
                                struct findt_alert_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);
                                
/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the FINDT.
 *
 ****************************************************************************************
 */
int app_findt_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct findt_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the FINDT.
 *
 ****************************************************************************************
 */
int app_findt_error_ind_handler(ke_msg_id_t const msgid,
                                struct prf_server_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

#endif  // BLE_FINDME_TARGET

/// @} APP_FINDT_TASK

#endif // APP_FINDT_TASK_H_
