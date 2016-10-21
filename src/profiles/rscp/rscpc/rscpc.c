/**
 ****************************************************************************************
 *
 * @file rscpc.c
 *
 * @brief Running Speed and Cadence Profile Collector implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 * $ Rev $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup RSCPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 #include "app_config.h"

#if (BLE_RSC_COLLECTOR)

#include "rscpc.h"
#include "rscpc_task.h"

/*
 * GLOBAL VARIABLES DECLARATION
 ****************************************************************************************
 */

struct rscpc_env_tag **rscpc_envs = NULL;

/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void rscpc_init(void)
{
    // Register RSCPC TASK into kernel
    task_rscpc_desc_register();

    // Reset all the profile role tasks
    PRF_CLIENT_RESET(rscpc_envs, RSCPC);
}

void rscps_send_no_conn_cmp_evt(uint8_t src_id, uint8_t dest_id, uint16_t conhdl, uint8_t operation)
{
    // Send the message
    struct rscpc_cmp_evt *evt = KE_MSG_ALLOC(RSCPC_CMP_EVT,
                                             dest_id, src_id,
                                             rscpc_cmp_evt);

    evt->conhdl     = conhdl;
    evt->operation  = operation;
    evt->status     = PRF_ERR_REQ_DISALLOWED;

    ke_msg_send(evt);
}

void rscpc_send_cmp_evt(struct rscpc_env_tag *rscpc_env, uint8_t operation, uint8_t status)
{
    // Free the stored operation if needed
    if (rscpc_env->operation != NULL)
    {
        ke_msg_free(ke_param2msg(rscpc_env->operation));
        rscpc_env->operation = NULL;
    }

    // Go back to the CONNECTED state if the state is busy
    if (ke_state_get(rscpc_env->con_info.prf_id) == RSCPC_BUSY)
    {
        ke_state_set(rscpc_env->con_info.prf_id, RSCPC_CONNECTED);
    }

    // Send the message
    struct rscpc_cmp_evt *evt = KE_MSG_ALLOC(RSCPC_CMP_EVT,
                                             rscpc_env->con_info.appid, rscpc_env->con_info.prf_id,
                                             rscpc_cmp_evt);

    evt->conhdl     = rscpc_env->con_info.conhdl;
    evt->operation  = operation;
    evt->status     = status;

    ke_msg_send(evt);
}

#endif //(BLE_RSC_COLLECTOR)

/// @} RSCP
