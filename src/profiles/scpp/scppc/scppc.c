/**
 ****************************************************************************************
 *
 * @file scppc.c
 *
 * @brief Scan Parameters Profile Client implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev$
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup SCPPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_SP_CLIENT)

#include "scppc.h"
#include "scppc_task.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

struct scppc_env_tag **scppc_envs;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void scppc_init(void)
{
	// Registet SCPPC task into kernel
	task_scppc_desc_register();

    // Reset all the profile role tasks
    PRF_CLIENT_RESET(scppc_envs, SCPPC);
}


void scppc_enable_cfm_send(struct scppc_env_tag *scppc_env, struct prf_con_info *con_info, uint8_t status)
{
    //send APP the details of the discovered attributes on SCPPC
    struct scppc_enable_cfm * rsp = KE_MSG_ALLOC(SCPPC_ENABLE_CFM,
                                                 con_info->appid, con_info->prf_id,
                                                 scppc_enable_cfm);

    rsp->conhdl = con_info->conhdl;
    rsp->status = status;

    if (status == PRF_ERR_OK)
    {
        rsp->scps = scppc_env->scps;

        // Go to connected state
        ke_state_set(con_info->prf_id, SCPPC_CONNECTED);

        // If Scan Refresh Char. has been discovered
        if (scppc_env->scps.chars[SCPPC_CHAR_SCAN_REFRESH].char_hdl != ATT_INVALID_HANDLE)
        {
            // Register SCPPC task in gatt for notifications
            prf_register_atthdl2gatt(con_info, &scppc_env->scps.svc);
        }
    }
    else
    {
        PRF_CLIENT_ENABLE_ERROR(scppc_envs, con_info->prf_id, SCPPC);
    }

    ke_msg_send(rsp);
}

void scppc_error_ind_send(struct scppc_env_tag *scppc_env, uint8_t status)
{
    struct scppc_error_ind *ind = KE_MSG_ALLOC(SCPPC_ERROR_IND,
                                               scppc_env->con_info.appid, scppc_env->con_info.prf_id,
                                               scppc_error_ind);

    ind->conhdl    = scppc_env->con_info.conhdl;
    ind->status    = status;

    // Send the message
    ke_msg_send(ind);
}

#endif /* (BLE_SP_CLIENT) */

/// @} SCPPC
