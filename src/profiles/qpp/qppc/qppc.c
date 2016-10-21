/**
 ****************************************************************************************
 *
 * @file qppc.c
 *
 * @brief Header file - Quintic Private Profile Client implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup QPPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_QPP_CLIENT)
#include "qppc.h"
#include "qppc_task.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
struct qppc_env_tag **qppc_envs;

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void qppc_init(void)
{
    // Register QPPC task into kernel
    task_qppc_desc_register();

    // Reset all the profile role tasks
    PRF_CLIENT_RESET(qppc_envs, QPPC);
}


void qppc_enable_cfm_send(struct qppc_env_tag *qppc_env, struct prf_con_info *con_info, uint8_t status)
{
    //send APP the details of the discovered attributes on QPPC
    struct qppc_enable_cfm * rsp = KE_MSG_ALLOC(QPPC_ENABLE_CFM,
                                                con_info->appid, con_info->prf_id,
                                                qppc_enable_cfm);

    rsp->conhdl = con_info->conhdl;
    rsp->status = status;

    if (status == PRF_ERR_OK)
    {
        rsp->qpps   = qppc_env->qpps;
        rsp->nb_ntf_char = qppc_env->nb_char - 1; // exclude one RX characteristic

        //register HRPC task in gatt for indication/notifications
        prf_register_atthdl2gatt(&qppc_env->con_info, &qppc_env->qpps.svc);

        // Go to connected state
        ke_state_set(qppc_env->con_info.prf_id, QPPC_CONNECTED);
    }
    else
    {
        PRF_CLIENT_ENABLE_ERROR(qppc_env, con_info->prf_id, QPPC);
    }
    
    ke_msg_send(rsp);
}

void qppc_error_ind_send(struct qppc_env_tag *qppc_env, uint8_t status)
{
    struct qppc_error_ind *ind = KE_MSG_ALLOC(QPPC_ERROR_IND,
                                              qppc_env->con_info.appid, qppc_env->con_info.prf_id,
                                              qppc_error_ind);

    ind->conhdl    = qppc_env->con_info.conhdl;
    //it will be an QPPC status code
    ind->status    = status;
    // send the message
    ke_msg_send(ind);
}

void qppc_disable(struct qppc_env_tag *qppc_env)
{
//    struct prf_client_disable_ind *ind = KE_MSG_ALLOC(QPPC_DISABLE_IND,
//                                                      qppc_env->con_info.appid, qppc_env->con_info.prf_id,
//                                                      prf_client_disable_ind);

//    ind->conhdl    = qppc_env->con_info.conhdl;
//    ind->status    = PRF_ERR_OK;

//    // Send the message
//    ke_msg_send(ind);

//    // Go to idle state
//    ke_state_set(qppc_env->con_info.prf_id, QPPC_IDLE);
}

#endif /* (BLE_QPP_CLIENT) */

/// @} QPPC
