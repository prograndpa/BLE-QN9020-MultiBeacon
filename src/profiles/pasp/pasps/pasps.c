/**
 ****************************************************************************************
 *
 * @file pasps.c
 *
 * @brief Phone Alert Status Profile Server implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup PASPS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_PAS_SERVER)

#include "atts_util.h"
#include "pasps.h"
#include "pasps_task.h"

/*
 * PHONE ALERT STATUS SERVICE ATTRIBUTES
 ****************************************************************************************
 */

/// Full PAS Database Description - Used to add attributes into the database
const struct atts_desc pasps_att_db[PASS_IDX_NB] =
{
    // Phone Alert Status Service Declaration
    [PASS_IDX_SVC]                   =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), sizeof(pasps_pass_svc),
                                          sizeof(pasps_pass_svc), (uint8_t *)&pasps_pass_svc},

    // Alert Status Characteristic Declaration
    [PASS_IDX_ALERT_STATUS_CHAR]     =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(pasps_alert_status_char),
                                          sizeof(pasps_alert_status_char), (uint8_t *)&pasps_alert_status_char},
    // Alert Status Characteristic Value
    [PASS_IDX_ALERT_STATUS_VAL]      =   {ATT_CHAR_ALERT_STATUS, PERM(RD, ENABLE) | PERM(NTF, ENABLE), sizeof(uint8_t),
                                          0, NULL},
    // Alert Status Characteristic - Client Characteristic Configuration Descriptor
    [PASS_IDX_ALERT_STATUS_CFG]      =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WR, ENABLE), sizeof(uint16_t),
                                          0, NULL},

    // Ringer Setting Characteristic Declaration
    [PASS_IDX_RINGER_SETTING_CHAR]   =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(pasps_ringer_setting_char),
                                          sizeof(pasps_ringer_setting_char), (uint8_t *)&pasps_ringer_setting_char},
    // Ringer Settings Characteristic Value
    [PASS_IDX_RINGER_SETTING_VAL]    =   {ATT_CHAR_RINGER_SETTING, PERM(RD, ENABLE) | PERM(NTF, ENABLE), sizeof(uint8_t),
                                          0, NULL},
    // Ringer Settings Characteristic - Client Characteristic Configuration Descriptor
    [PASS_IDX_RINGER_SETTING_CFG]    =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WR, ENABLE), sizeof(uint16_t),
                                          0, NULL},

    // Ringer Control Point Characteristic Declaration
    [PASS_IDX_RINGER_CTNL_PT_CHAR]   =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(pasps_ringer_ctnl_pt_char),
                                          sizeof(pasps_ringer_ctnl_pt_char), (uint8_t *)&pasps_ringer_ctnl_pt_char},
    // Ringer Control Point Characteristic Value
    [PASS_IDX_RINGER_CTNL_PT_VAL]    =   {ATT_CHAR_RINGER_CNTL_POINT, PERM(WR, ENABLE), sizeof(uint8_t),
                                          0, NULL},
};

/// Phone Alert Status Service
const atts_svc_desc_t pasps_pass_svc = ATT_SVC_PHONE_ALERT_STATUS;

/// Phone Alert Status Service Alert Status Characteristic
const struct atts_char_desc pasps_alert_status_char   = ATTS_CHAR(ATT_CHAR_PROP_RD | ATT_CHAR_PROP_NTF, 0,
                                                                  ATT_CHAR_ALERT_STATUS);

/// Phone Alert Status Service Ringer Setting Characteristic
const struct atts_char_desc pasps_ringer_setting_char = ATTS_CHAR(ATT_CHAR_PROP_RD | ATT_CHAR_PROP_NTF, 0,
                                                                  ATT_CHAR_RINGER_SETTING);

/// Phone Alert Status Service Ringer Settings Characteristic
const struct atts_char_desc pasps_ringer_ctnl_pt_char = ATTS_CHAR(ATT_CHAR_PROP_WR_NO_RESP, 0,
                                                                  ATT_CHAR_RINGER_CNTL_POINT);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Phone Alert Status Profile Server Environment Variable
struct pasps_env_tag pasps_env;
/// Phone Alert Status Profile Server Environment Variable
struct pasps_idx_env_tag **pasps_idx_envs;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void pasps_init(void)
{
    // Reset Common Environment
    memset(&pasps_env, 0, sizeof(struct pasps_env_tag));

    // Register PASPS TASK into kernel
    task_pasps_desc_register();

    // Reset all environments
    prf_client_reset((prf_env_struct ***)&pasps_idx_envs, TASK_PASPS, PASPS_DISABLED);
}

void pasps_send_cmp_evt(ke_task_id_t src_id, ke_task_id_t dest_id, uint16_t conhdl,
                        uint8_t operation, uint8_t status)
{
    // Come back to the Connected state if the state was busy.
    if (ke_state_get(src_id) == PASPS_BUSY)
    {
        ke_state_set(src_id, PASPS_CONNECTED);
    }

    // Send the message to the application
    struct pasps_cmp_evt *evt = KE_MSG_ALLOC(PASPS_CMP_EVT,
                                             dest_id, src_id,
                                             pasps_cmp_evt);

    evt->conhdl      = conhdl;
    evt->operation   = operation;
    evt->status      = status;

    ke_msg_send(evt);
}

void pasps_disable(struct pasps_idx_env_tag *idx_env)
{
    // Disable PAS service
    attsdb_svc_set_permission(pasps_env.pass_shdl, PERM(SVC, DISABLE));

    struct pasps_disable_ind *ind = KE_MSG_ALLOC(PASPS_DISABLE_IND,
                                                 idx_env->con_info.appid, idx_env->con_info.prf_id,
                                                 pasps_disable_ind);

    memset(ind, 0x00, sizeof(struct pasps_disable_ind));

    ind->conhdl = idx_env->con_info.conhdl;

    if (PASPS_IS_NTF_ENABLED(idx_env, PASPS_FLAG_ALERT_STATUS_CFG))
    {
        ind->alert_status_ntf_cfg = PRF_CLI_START_NTF;
    }

    if (PASPS_IS_NTF_ENABLED(idx_env, PASPS_FLAG_RINGER_SETTING_CFG))
    {
        ind->ringer_setting_ntf_cfg = PRF_CLI_START_NTF;
    }

    ke_msg_send(ind);

    // Go to idle state
    ke_state_set(idx_env->con_info.prf_id, PASPS_IDLE);

    // Free the environment allocated for this connection
    prf_client_disable((prf_env_struct ***)&pasps_idx_envs, KE_IDX_GET(idx_env->con_info.prf_id));
}

#endif //(BLE_PASP_SERVER)

/// @} PASPS
