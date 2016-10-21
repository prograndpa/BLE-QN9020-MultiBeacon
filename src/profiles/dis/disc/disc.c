/**
 ****************************************************************************************
 *
 * @file disc.c
 *
 * @brief Device Information Service Client Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 5767 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup DISC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_DIS_CLIENT)
#include "disc.h"
#include "disc_task.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct disc_env_tag **disc_envs;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void disc_init(void)
{
	// Register DISC task into kernel
    task_disc_desc_register();

    // Reset all the profile role tasks
    PRF_CLIENT_RESET(disc_envs, DISC);
}

void disc_read_char_rsp_send(struct disc_env_tag *disc_env,
                             struct gatt_read_char_resp const *param, uint8_t status)
{
    // Received value length
    uint8_t length = (param != NULL) ? param->data.len : 0;

    struct disc_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(DISC_RD_CHAR_RSP,
                                                    disc_env->con_info.appid, disc_env->con_info.prf_id,
                                                    disc_rd_char_rsp, length);

    rsp->conhdl     = disc_env->con_info.conhdl;
    rsp->status     = status;
    rsp->char_code  = disc_env->last_char_code;
    rsp->val_length = length;

    if (param != NULL)
    {
        memcpy(&rsp->val, &param->data.data[0], length);
    }

    ke_msg_send(rsp);
}

void disc_enable_cfm_send(struct disc_env_tag *disc_env, struct prf_con_info *con_info, uint8_t status)
{
    // Send APP the details of the discovered attributes on DISS
    struct disc_enable_cfm * rsp = KE_MSG_ALLOC(DISC_ENABLE_CFM,
                                                con_info->appid, con_info->prf_id,
                                                disc_enable_cfm);

    rsp->conhdl = con_info->conhdl;
    rsp->status = status;

    if (status == PRF_ERR_OK)
    {
        rsp->dis = disc_env->dis;

        // Go to connected state
        ke_state_set(con_info->prf_id, DISC_CONNECTED);
    }
    else
    {
        PRF_CLIENT_ENABLE_ERROR(disc_envs, con_info->prf_id, DISC);
    }

    ke_msg_send(rsp);
}

#endif //BLE_DIS_CLIENT
/// @} DISC
