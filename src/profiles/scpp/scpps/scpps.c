/**
 ****************************************************************************************
 *
 * @file scpps.c
 *
 * @brief Scan Parameters Profile Server implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev$
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SCPPS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_SP_SERVER)

#include "atts_db.h"
#include "scpps.h"
#include "scpps_task.h"

/*
 * SCPP ATTRIBUTES DEFINITION
 ****************************************************************************************
 */

/// Full SCPS Database Description - Used to add attributes into the database
const struct atts_desc scpps_att_db[SCPS_IDX_NB] =
{
    // Scan Parameters Service Declaration
    [SCPS_IDX_SVC]                      =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), sizeof(scps_svc),
                                             sizeof(scps_svc), (uint8_t *)&scps_svc},

    // Scan Interval Window Characteristic Declaration
    [SCPS_IDX_SCAN_INTV_WD_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(scps_scan_intv_wd_char),
                                             sizeof(scps_scan_intv_wd_char), (uint8_t *)&scps_scan_intv_wd_char},
    // Scan Interval Window Characteristic Value
    [SCPS_IDX_SCAN_INTV_WD_VAL]         =   {ATT_CHAR_SCAN_INTV_WD, PERM(WR, ENABLE), sizeof(struct scan_intv_wd),
                                             0, NULL},

    // Scan Refresh Characteristic Declaration
    [SCPS_IDX_SCAN_REFRESH_CHAR]        =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(scps_scan_refresh_char),
                                             sizeof(scps_scan_refresh_char), (uint8_t *)&scps_scan_refresh_char},
    // Scan Refresh Characteristic Value
    [SCPS_IDX_SCAN_REFRESH_VAL]         =   {ATT_CHAR_SCAN_REFRESH, PERM(NTF, ENABLE), sizeof(uint8_t),
                                             0, NULL},
    // Scan Refresh Characteristic - Client Characteristic Configuration Descriptor
    [SCPS_IDX_SCAN_REFRESH_NTF_CFG]     =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WR, ENABLE), sizeof(uint16_t),
                                             0, NULL},
};

/// Scan Parameters Service
const atts_svc_desc_t scps_svc = ATT_SVC_SCAN_PARAMETERS;

/// Scan Parameters Service Scan Interval Window characteristic
const struct atts_char_desc scps_scan_intv_wd_char = ATTS_CHAR(ATT_CHAR_PROP_WR_NO_RESP,
                                                               0,
                                                               ATT_CHAR_SCAN_INTV_WD);

/// Scan Parameters Service Scan Refresh characteristic
const struct atts_char_desc scps_scan_refresh_char = ATTS_CHAR(ATT_CHAR_PROP_NTF,
                                                               0,
                                                               ATT_CHAR_SCAN_REFRESH);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct scpps_env_tag scpps_env;

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void scpps_init(void)
{
    // Reset the  environment
    memset(&scpps_env, 0, sizeof(scpps_env));
	
	// Registet SCPPS task into kernel
	task_scpps_desc_register();

    // Go to IDLE state
    ke_state_set(TASK_SCPPS, SCPPS_DISABLED);
}

void scpps_scan_refresh_cfm_send(uint8_t status)
{
    struct scpps_scan_refresh_send_cfm * cfm = KE_MSG_ALLOC(SCPPS_SCAN_REFRESH_SEND_CFM,
                                                            scpps_env.con_info.appid,
                                                            TASK_SCPPS, scpps_scan_refresh_send_cfm);

    cfm->conhdl = scpps_env.con_info.conhdl;
    cfm->status = status;

    ke_msg_send(cfm);
}

void scpps_disable(void)
{
    //Disable SCPS in database
    attsdb_svc_set_permission(scpps_env.shdl, PERM_RIGHT_DISABLE);

    //Send current configuration to the application
    struct scpps_disable_ind *ind = KE_MSG_ALLOC(SCPPS_DISABLE_IND,
                                                 scpps_env.con_info.appid, TASK_SCPPS,
                                                 scpps_disable_ind);

    ind->conhdl = scpps_env.con_info.conhdl;

    if (scpps_env.scan_refresh_ntf_cfg == PRF_CLI_START_NTF)
    {
        ind->scan_refresh_ntf_en = PRF_CLI_START_NTF;
    }

    ke_msg_send(ind);

    // Reset saved NTF cfg value
    scpps_env.scan_refresh_ntf_cfg = 0;

    // Go to Idle state
    ke_state_set(TASK_SCPPS, SCPPS_IDLE);
}

#endif // (BLE_SP_SERVER)

/// @} SCPPS
