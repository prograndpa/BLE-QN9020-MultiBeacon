/**
 ****************************************************************************************
 *
 * @file rscps.c
 *
 * @brief Running Speed and Cadence Profile Sensor implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup RSCPS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_RSC_SENSOR)

#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "rscps.h"
#include "rscps_task.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct rscps_env_tag rscps_env;

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void rscps_init(void)
{
    // Reset environment
    memset(&rscps_env, 0, sizeof(rscps_env));
    
    // Register RSCPS TASK into kernel
    task_rscps_desc_register();

    // Go to IDLE state
    ke_state_set(TASK_RSCPS, RSCPS_DISABLED);
}

void rscps_send_rsp_ind(uint16_t handle, uint8_t req_op_code, uint8_t status)
{
    // Error response
    uint8_t rsp[RSCP_SC_CNTL_PT_RSP_MIN_LEN];

    // Response Code
    rsp[0] = RSCP_CTNL_PT_RSP_CODE;
    // Request Operation Code
    rsp[1] = req_op_code;
    // Response value
    rsp[2] = status;

    // Set the value in the database - If we are here the handle is valid
    attsdb_att_set_value(handle, 3, (uint8_t *)&rsp);

    // Send an indication
    struct gatt_indicate_req *ind = KE_MSG_ALLOC(GATT_INDICATE_REQ,
                                                 TASK_GATT, rscps_env.con_info.prf_id,
                                                 gatt_indicate_req);

    ind->conhdl  = rscps_env.con_info.conhdl;
    ind->charhdl = handle;

    ke_msg_send(ind);
}

void rscps_send_cmp_evt(uint8_t src_id, uint8_t dest_id, uint16_t conhdl,
                        uint8_t operation, uint8_t status)
{
    // Go back to the Connected state if the state is busy
    if (ke_state_get(src_id) == RSCPS_BUSY)
    {
        ke_state_set(src_id, RSCPS_CONNECTED);
    }

    // Set the operation code
    rscps_env.operation = RSCPS_RESERVED_OP_CODE;

    // Send the message
    struct rscps_cmp_evt *evt = KE_MSG_ALLOC(RSCPS_CMP_EVT,
                                             dest_id, src_id,
                                             rscps_cmp_evt);

    evt->conhdl     = conhdl;
    evt->operation  = operation;
    evt->status     = status;

    ke_msg_send(evt);
}

void rscps_disable(void)
{
	// Disable RSCS in database
    attsdb_svc_set_permission(rscps_env.shdl, PERM_RIGHT_DISABLE);

    //Send current configuration to APP
    struct rscps_disable_ind* ind = KE_MSG_ALLOC(RSCPS_DISABLE_IND,
                                                 rscps_env.con_info.appid, rscps_env.con_info.prf_id,
                                                 rscps_disable_ind);

    ind->conhdl = rscps_env.con_info.conhdl;

    // RSC Measurement Char. - Client Characteristic Configuration Descriptor
    ind->rsc_meas_ntf_cfg = RSCPS_IS_NTFIND_ENABLED(RSCP_PRF_CFG_FLAG_RSC_MEAS_NTF)
                            ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    // SC Control Point Char. - Client Characteristic Configuration Descriptor
    if (rscps_env.hdl_offset[RSCP_RSCS_SC_CTNL_PT_CHAR] != 0)
    {
        ind->sc_ctnl_pt_ntf_cfg = RSCPS_IS_NTFIND_ENABLED(RSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND)
                                  ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND;
    }

    ke_msg_send(ind);

    // Reset the notification/indication status
    rscps_env.prf_cfg &= RSCP_FEAT_ALL_SUPP;

    // Go to idle state
    ke_state_set(TASK_RSCPS, RSCPS_IDLE);
}

#endif //(BLE_RSC_SENSOR)

/// @} RSCPS
