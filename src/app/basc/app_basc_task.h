/**
 ****************************************************************************************
 *
 * @file app_basc.h
 *
 * @brief Application BASC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BASC_TASK_H_
#define APP_BASC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_BASC_TASK Battery Service Client Task API
 * @ingroup APP_BASC
 * @brief Battery Service Client Task API
 *
 * Battery Service Client Task APIs are used to handle the message from BASC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_BATT_CLIENT
#include "app_basc.h"
/// @cond

/// Battery Service Client environment variable
struct app_basc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
    ///Number of BAS instances found
    uint8_t bas_nb;

    uint8_t cur_code;    
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_basc_env_tag *app_basc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the BASC.
 *
 ****************************************************************************************
 */
int app_basc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct basc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the BASC.
 *
 ****************************************************************************************
 */
int app_basc_error_ind_handler(ke_msg_id_t const msgid,
                      struct basc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_basc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct basc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Inform APP that Battery Level Notification Configuration has been read
 *
 ****************************************************************************************
 */
int app_basc_batt_level_ntf_cfg_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct basc_batt_level_ntf_cfg_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Inform APP that Battery Level Characteristic Presentation Format has been read
 *
 ****************************************************************************************
 */
int app_basc_batt_level_pres_format_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct basc_batt_level_pres_format_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Indicate to APP that the Battery Level value has been received.
 *
 ****************************************************************************************
 */
int app_basc_batt_level_ind_handler(ke_msg_id_t const msgid,
                      struct basc_batt_level_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP.
 *
 ****************************************************************************************
 */
int app_basc_disable_ind_handler(ke_msg_id_t const msgid,
                          struct prf_client_disable_ind *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

#endif // BLE_BATT_CLIENT

/// @} APP_BASC_TASK

#endif // APP_BASC_TASK_H_
