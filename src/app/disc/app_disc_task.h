/**
 ****************************************************************************************
 *
 * @file app_disc_task.h
 *
 * @brief Application DISC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_DISC_TASK_H_
#define APP_DISC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_DISC_TASK Device Information Service Client Task API
 * @ingroup APP_DISC
 * @brief Device Information Service Client Task API
 *
 * Device Information Service Client Task APIs are used to handle the message from DISC or APP
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_DIS_CLIENT
#include "app_disc.h"
/// @cond

/// Device Information Profile Client environment variable
struct app_disc_env_tag
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
extern struct app_disc_env_tag *app_disc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the DISC.
 *
 ****************************************************************************************
 */
int app_disc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct disc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 *
 ****************************************************************************************
 */
int app_disc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct disc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_disc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

#endif // BLE_DIS_CLIENT

/// @} APP_DISC_TASK

#endif // APP_DISC_TASK_H_
