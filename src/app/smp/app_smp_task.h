/**
 ****************************************************************************************
 *
 * @file app_smp_task.h
 *
 * @brief Application SMP task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_SMP_TASK_H_
#define APP_SMP_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_SMP_TASK  Security Manager Protocol Task API
 * @ingroup APP_SMP
 * @brief  Security Manager Protocol Task API
 *
 * SMP Task APIs are used to handle the message from SMPM, SMPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Set key confirmation from the SMPM.
 *
 ****************************************************************************************
 */
int app_smpm_set_key_cfm_handler(ke_msg_id_t const msgid, struct smpm_set_key_cfm const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Key indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_key_ind_handler(ke_msg_id_t const msgid, struct smpc_key_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief TK request indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_tk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_tk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief LTK request indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_ltk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_ltk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief IRK request indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_irk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_irk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief CSRK request indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_csrk_req_ind_handler(ke_msg_id_t const msgid, struct smpc_csrk_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Check BD address request indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_chk_bd_addr_req_ind_handler(ke_msg_id_t const msgid, struct smpc_chk_bd_addr_req const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Encryption procedure complete indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_sec_started_ind_handler(ke_msg_id_t const msgid, struct smpc_sec_started_ind const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Timeout indication from the SMPC.
 *
 ****************************************************************************************
 */
int app_smpc_timeout_evt_handler(ke_msg_id_t const msgid, struct smpc_timeout_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
/// @} APP_SMP_TASK
#endif // APP_SMP_TASK_H_
