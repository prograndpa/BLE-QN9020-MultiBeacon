/**
 ****************************************************************************************
 *
 * @file proxm.c
 *
 * @brief Proximity monitor implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup PROXM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_PROX_MONITOR)

#include "gatt_task.h"
#include "proxm.h"
#include "proxm_task.h"
#include "prf_utils.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct proxm_env_tag **proxm_envs;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void proxm_init(void)
{
    // Register PROXM task into kernel 
    task_proxm_desc_register();
    // Reset all the profile role tasks
    PRF_CLIENT_RESET(proxm_envs, PROXM);
}

void proxm_enable_cfm_send(struct proxm_env_tag *proxm_env, struct prf_con_info *con_info, uint8_t status)
{
    //format response to app
    struct proxm_enable_cfm * cfm = KE_MSG_ALLOC(PROXM_ENABLE_CFM,
                                                 con_info->appid, con_info->prf_id,
                                                 proxm_enable_cfm);

    cfm->conhdl = con_info->conhdl;
    cfm->status = status;

    if (status == PRF_ERR_OK)
    {
        cfm->ias    = proxm_env->ias;
        cfm->lls    = proxm_env->lls;
        cfm->txps   = proxm_env->txps;

        // Go to connected state
        ke_state_set(con_info->prf_id, PROXM_CONNECTED);
    }
    else
    {
        PRF_CLIENT_ENABLE_ERROR(proxm_envs, con_info->prf_id, PROXM);
    }

    ke_msg_send(cfm);
}

void proxm_read_char_rsp_send(struct proxm_env_tag *proxm_env,
                              struct gatt_read_char_resp const *param, uint8_t status)
{
    // Received value length
    uint8_t length = (param != NULL) ? param->data.len : 0;

    struct proxm_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(PROXM_RD_CHAR_RSP,
                                                     proxm_env->con_info.appid, proxm_env->con_info.prf_id,
                                                     proxm_rd_char_rsp, length);

    rsp->conhdl     = proxm_env->con_info.conhdl;
    rsp->status     = status;
    rsp->char_code  = proxm_env->last_char_code;

    if (param != NULL)
    {
        memcpy(&rsp->val, &param->data.data, length);
    }

    ke_msg_send(rsp);
}

void proxm_write_char_rsp_send(struct proxm_env_tag *proxm_env, uint8_t status)
{
    // Send response to app
    struct proxm_wr_char_rsp *wr_cfm = KE_MSG_ALLOC(PROXM_WR_CHAR_RSP,
                                                    proxm_env->con_info.appid, proxm_env->con_info.prf_id,
                                                    proxm_wr_char_rsp);

    wr_cfm->conhdl    = proxm_env->con_info.conhdl;
    wr_cfm->status    = status;

    ke_msg_send(wr_cfm);
}

#endif //BLE_PROX_MONITOR

/// @} PROXM
