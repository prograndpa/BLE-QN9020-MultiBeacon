/**
 ****************************************************************************************
 *
 * @file app_findl_task.h
 *
 * @brief Application FMPL task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_FINDL_TASK_H_
#define APP_FINDL_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_FINDL_TASK Find Me Locator Task API
 * @ingroup APP_FINDL
 * @brief Find Me Locator Task API
 *
 * Find Me Locator Task APIs are used to handle the message from FINDL or APP.
 * @{
 ****************************************************************************************
 */


#if BLE_FINDME_LOCATOR

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_findl.h"
/// @cond

/// Find Me Profile Locator environment variable
struct app_findl_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_findl_env_tag *app_findl_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the FINDL.
 *
 ****************************************************************************************
 */
int app_findl_enable_cfm_handler(ke_msg_id_t const msgid,
                                 struct findl_enable_cfm *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the FINDL.
 *
 ****************************************************************************************
 */
int app_findl_error_ind_handler(ke_msg_id_t const msgid,
                                struct findl_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_findl_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);
                      
#endif // BLE_FINDME_LOCATOR

/// @} APP_FINDL_TASK

#endif // APP_FINDL_TASK_H_
