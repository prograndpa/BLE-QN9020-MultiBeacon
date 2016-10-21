/**
 ****************************************************************************************
 *
 * @file findt.c
 *
 * @brief Find Me Target implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup FINDT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_FINDME_TARGET)
#include "findt.h"
#include "findt_task.h"

/*
 * FINDT PROFILE ATTRIBUTES
 ****************************************************************************************
 */
/// Full IAS Database Description - Used to add attributes into the database
const struct atts_desc findt_att_db[FINDT_IAS_IDX_NB] =
{
    // Immediate Alert Service Declaration
    [FINDT_IAS_IDX_SVC]              =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), sizeof(findt_ias_svc),
                                         sizeof(findt_ias_svc), (uint8_t *)&findt_ias_svc},

    // Alert Level Characteristic Declaration
    [FINDT_IAS_IDX_ALERT_LVL_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(findt_alert_lvl_char),
                                         sizeof(findt_alert_lvl_char), (uint8_t *)&findt_alert_lvl_char},
    // Alert Level Characteristic Value
    [FINDT_IAS_IDX_ALERT_LVL_VAL]   =   {ATT_CHAR_ALERT_LEVEL, PERM(WR, ENABLE), sizeof(uint8_t),
                                         0, NULL},
};

/*
 *  FIND ME PROFILE TARGET ATTRIBUTES
 ****************************************************************************************
 */
/// Immediate Alert Service
const atts_svc_desc_t findt_ias_svc = ATT_SVC_IMMEDIATE_ALERT;

/// Alert Level Service - Heart Rate Measurement Characteristic
const struct atts_char_desc findt_alert_lvl_char = ATTS_CHAR(ATT_CHAR_PROP_WR_NO_RESP,
                                                               0,
                                                             ATT_CHAR_ALERT_LEVEL);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct findt_env_tag findt_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void findt_init(void)
{
    // Reset environment
    memset(&findt_env, 0, sizeof(findt_env));
    
    // Register FINDT task into kernel  
    task_findt_desc_register();

    // Go to IDLE state
    ke_state_set(TASK_FINDT, FINDT_DISABLED);
}

void findt_disable(void)
{
    // Inform the application about the disconnection
    struct findt_disable_ind *ind = KE_MSG_ALLOC(FINDT_DISABLE_IND,
                                                 findt_env.con_info.appid, TASK_FINDT,
                                                 findt_disable_ind);

    ind->conhdl = findt_env.con_info.conhdl;

    ke_msg_send(ind);

    //Disable IAS in database
    attsdb_svc_set_permission(findt_env.shdl, PERM_RIGHT_DISABLE);

    //Go to idle state
    ke_state_set(TASK_FINDT, FINDT_IDLE);
}


#endif //BLE_FINDME_TARGET

/// @} FINDT
