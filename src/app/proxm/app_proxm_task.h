/**
 ****************************************************************************************
 *
 * @file app_proxm_task.h
 *
 * @brief Application PROXM task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_PROXM_TASK_H_
#define APP_PROXM_TASK_H_


/**
 ****************************************************************************************
 * @addtogroup APP_PROXM_TASK Proximity Monitor Task API
 * @ingroup APP_PROXM
 * @brief Proximity Monitor Task API
 *
 * Proximity Monitor Task APIs are used to handle the message from PROXM or APP. 
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_PROX_MONITOR
#include "app_proxm.h"
/// @cond

/// Proximity Monitor Profile environment variable
struct app_proxm_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
    uint8_t cur_code;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_proxm_env_tag *app_proxm_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the PROXM.
 *
 ****************************************************************************************
 */
int app_proxm_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct proxm_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 *
 ****************************************************************************************
 */
int app_proxm_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct proxm_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_proxm_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct proxm_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP
 *
 ****************************************************************************************
 */
int app_proxm_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

#endif // BLE_PROX_MONITOR

/// @} APP_PROXM_TASK

#endif // APP_PROXM_TASK_H_
