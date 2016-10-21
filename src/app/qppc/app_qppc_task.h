/**
 ****************************************************************************************
 *
 * @file app_qppc_task.h
 *
 * @brief Application QPPC task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef APP_QPPC_TASK_H_
#define APP_QPPC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_QPPC_TASK Quintic Private Profile Client Task API
 * @ingroup APP_QPPC
 * @brief Quintic Private Profile Client Task API
 *
 * Private Profile Client Task APIs are used to handle the message from QPPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_QPP_CLIENT
#include "app_qppc.h"
/// @cond

/// environment variable
struct app_qppc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
#if QN_SVC_CONTENT_USED	
    /// Handle values and characteristic properties
    struct qpps_content qpps;
#endif
    /// Number of characteristic with notify property
    uint8_t nb_ntf_char;
    /// Characteristic operation code
    uint8_t cur_code;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_qppc_env_tag *app_qppc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the QPPC.
 *
 ****************************************************************************************
 */
int app_qppc_enable_cfm_handler(ke_msg_id_t const msgid,
                                struct qppc_enable_cfm *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the QPPC.
 *
 ****************************************************************************************
 */
int app_qppc_error_ind_handler(ke_msg_id_t const msgid,
                               struct qppc_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 *
 ****************************************************************************************
 */
int app_qppc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                                 struct qppc_rd_char_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_qppc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                                 struct qppc_wr_char_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the value send to APP.
 *
 ****************************************************************************************
 */
int app_qppc_data_ind_handler(ke_msg_id_t const msgid,
                              struct qppc_data_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the QPPC disable indication.
 *
 ****************************************************************************************
 */
int app_qppc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

#endif // BLE_QPP_CLIENT

/// @} APP_QPPC_TASK

#endif // APP_QPPC_TASK_H_
