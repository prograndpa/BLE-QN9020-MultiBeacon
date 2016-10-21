/**
 ****************************************************************************************
 *
 * @file findl.c
 *
 * @brief Find Me Locator implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup FINDL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_FINDME_LOCATOR)
#include "findl.h"
#include "findl_task.h"
#include "prf_utils.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct findl_env_tag **findl_envs;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void findl_init(void)
{
    // Register FINDL task into kernel
    task_findl_desc_register();
	
    // Reset all the profile role tasks
    PRF_CLIENT_RESET(findl_envs, FINDL);
}

void findl_enable_cfm_send(struct findl_env_tag *findl_env, struct prf_con_info *con_info, uint8_t status)
{
    //send response to app
    struct findl_enable_cfm * cfm = KE_MSG_ALLOC(FINDL_ENABLE_CFM,
                                                 con_info->appid, con_info->prf_id,
                                                 findl_enable_cfm);

    cfm->conhdl = con_info->conhdl;
    cfm->status = status;

    if (status == PRF_ERR_OK)
    {
        cfm->ias = findl_env->ias;

        // Go to connected state
        ke_state_set(con_info->prf_id, FINDL_CONNECTED);
    }
    else
    {
        PRF_CLIENT_ENABLE_ERROR(findl_envs, con_info->prf_id, FINDL);
    }

    ke_msg_send(cfm);
}

void findl_error_ind_send(struct findl_env_tag *findl_env, uint8_t status)
{
    struct findl_error_ind *ind = KE_MSG_ALLOC(FINDL_ERROR_IND,
                                               findl_env->con_info.appid, findl_env->con_info.prf_id,
                                               findl_error_ind);

    ind->conhdl    = findl_env->con_info.conhdl;
    // It will be a FINDL status code
    ind->status    = status;

    // Send the message
    ke_msg_send(ind);
}

#endif //BLE_FINDME_LOCATOR

/// @} FINDL
