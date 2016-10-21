/**
 ****************************************************************************************
 *
 * @file scpps_task.c
 *
 * @brief Scan Parameters Profile Server Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev$
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup SCPPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_SP_SERVER)
#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"

#include "scpps.h"
#include "scpps_task.h"
#include "scpp_common.h"

#include "prf_utils.h"

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SCPPS_CREATE_DB_REQ message.
 * The handler adds SCPS into the database using value of the features param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int scpps_create_db_req_handler(ke_msg_id_t const msgid,
                                       struct scpps_create_db_req const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Service Configuration Flag
    uint8_t cfg_flag = SCPPS_CFG_FLAG_MANDATORY_MASK;
    // Database Creation Status
    uint8_t status;

    // Save Profile ID
    scpps_env.con_info.prf_id = TASK_SCPPS;
    // Save Database Configuration
    scpps_env.features = param->features;

    /*---------------------------------------------------*
     * Scan Parameters Service Creation
     *---------------------------------------------------*/
    // Set Configuration Flag Value
    if (param->features == SCPPS_SCAN_REFRESH_CHAR_SUP)
    {
        cfg_flag |= SCPPS_CFG_FLAG_SCAN_REFRESH_MASK;
    }

    // Add Service Into Database
    status = atts_svc_create_db(&scpps_env.shdl, &cfg_flag, SCPS_IDX_NB, NULL,
                                dest_id, &scpps_att_db[0]);
    // Disable SCPPS
    attsdb_svc_set_permission(scpps_env.shdl, PERM(SVC, DISABLE));


    // Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        // If we are here, database has been fulfilled with success, go to idle state
        ke_state_set(TASK_SCPPS, SCPPS_IDLE);
    }

    // Send response to application
    struct scpps_create_db_cfm * cfm = KE_MSG_ALLOC(SCPPS_CREATE_DB_CFM, src_id,
                                                    TASK_SCPPS, scpps_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SCPPS_ENABLE_REQ message.
 * The handler enables the Scan Parameters Profile Server Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int scpps_enable_req_handler(ke_msg_id_t const msgid,
                                    struct scpps_enable_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Value used to initialize NTF Configuration value in DB (only readable att)
    uint16_t ntf_cfg = 0;

    // Save the application task id
    scpps_env.con_info.appid = src_id;
    // Save the connection handle associated to the profile
    scpps_env.con_info.conhdl = param->conhdl;

    // Check if the provided connection exist
    if (gap_get_rec_idx(param->conhdl) == GAP_INVALID_CONIDX)
    {
        // The connection doesn't exist, request disallowed
        prf_server_error_ind_send((prf_env_struct *)&scpps_env, PRF_ERR_REQ_DISALLOWED,
                                  SCPPS_ERROR_IND, SCPPS_ENABLE_REQ);
    }
    else
    {
        // Check if Scan Refresh Characteristic is supported
        if (scpps_env.features == SCPPS_SCAN_REFRESH_CHAR_SUP)
        {
            // If this connection is a not configuration one, apply config saved by app
            if(param->con_type == PRF_CON_NORMAL)
            {
                // Copy provided value in ntf_cfg
                memcpy(&ntf_cfg, &param->scan_refresh_ntf_en, sizeof(uint16_t));

                // Update state value saved in the environment - Only 1st byte is meaningful
                scpps_env.scan_refresh_ntf_cfg = (uint8_t)ntf_cfg;
            }

            //Set Scan Refresh Char. NTF Configuration in DB - 0 if not normal connection
            attsdb_att_set_value(scpps_env.shdl + SCPS_IDX_SCAN_REFRESH_NTF_CFG, sizeof(uint16_t),
                                 (uint8_t *)&ntf_cfg);
        }

        // Enable Service + Set Security Level
        attsdb_svc_set_permission(scpps_env.shdl, param->sec_lvl);

        // Go to connected state
        ke_state_set(TASK_SCPPS, SCPPS_CONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SCPPS_SCAN_REFRESH_SEND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int scpps_scan_refresh_send_req_handler(ke_msg_id_t const msgid,
                                               struct scpps_scan_refresh_send_req const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
    uint8_t status = PRF_ERR_INVALID_PARAM;

    // Check Connection handle and Scan Refresh Value
    if ((param->conhdl == scpps_env.con_info.conhdl) ||
        (param->scan_refresh != SCPP_SERVER_REQUIRES_REFRESH))
    {
        // Check if Scan Refresh characteristic is supported
        if (scpps_env.features == SCPPS_SCAN_REFRESH_CHAR_SUP)
        {
            // Check if notifications are enabled
            if (scpps_env.scan_refresh_ntf_cfg == PRF_CLI_START_NTF)
            {
                // Update value in DB
                attsdb_att_set_value(scpps_env.shdl + SCPS_IDX_SCAN_REFRESH_VAL, sizeof(uint8_t),
                                     (uint8_t *)&param->scan_refresh);

                // Send notification through GATT
                struct gatt_notify_req * ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT,
                                                            TASK_SCPPS, gatt_notify_req);

                ntf->conhdl  = scpps_env.con_info.conhdl;
                ntf->charhdl = scpps_env.shdl + SCPS_IDX_SCAN_REFRESH_VAL;

                ke_msg_send(ntf);

                // Update status for cfm to APP
                status = PRF_ERR_OK;
            }
            // Notification not enabled, simply don't send anything
            else
            {
                status = PRF_ERR_NTF_DISABLED;
            }
        }
        else
        {
            status = PRF_ERR_FEATURE_NOT_SUPPORTED;
        }
    }

    if (status != PRF_ERR_OK)
    {
        scpps_scan_refresh_cfm_send(status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gatt_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Scan Interval Window value
    struct scan_intv_wd scan_intv_wd;
    // Notification Configuration value
    uint16_t ntf_cfg;
    // Status for write response
    uint8_t status = PRF_ERR_OK;

    // Scan Interval Window Value
    if (param->handle == scpps_env.shdl + SCPS_IDX_SCAN_INTV_WD_VAL)
    {
        // Extract value
        memcpy(&scan_intv_wd.le_scan_intv,      &param->value[0], sizeof(uint16_t));
        memcpy(&scan_intv_wd.le_scan_window,    &param->value[2], sizeof(uint16_t));

        // Inform APP of Scan Interval Window change
        struct scpps_scan_intv_wd_ind * ind = KE_MSG_ALLOC(SCPPS_SCAN_INTV_WD_IND,
                                                           scpps_env.con_info.appid, TASK_SCPPS,
                                                           scpps_scan_intv_wd_ind);

        memcpy(&ind->conhdl, &scpps_env.con_info.conhdl, sizeof(uint16_t));
        memcpy(&ind->scan_intv_wd, &scan_intv_wd, sizeof(struct scan_intv_wd));

        ke_msg_send(ind);
    }
    // Scan Refresh Notification Configuration
    else if (param->handle == scpps_env.shdl + SCPS_IDX_SCAN_REFRESH_NTF_CFG)
    {
        // Check if Scan Refresh Characteristic is supported
        if (scpps_env.features == SCPPS_SCAN_REFRESH_CHAR_SUP)
        {
            // Extract value before check
            memcpy(&ntf_cfg, &param->value[0], sizeof(uint16_t));

            // Only update configuration if value for stop or notification enable
            if ((ntf_cfg == PRF_CLI_STOP_NTFIND) || (ntf_cfg == PRF_CLI_START_NTF))
            {
                // Update state value in environment
                scpps_env.scan_refresh_ntf_cfg = (uint8_t)ntf_cfg;

                //Update Scan Refresh Char. NTF Configuration in DB
                attsdb_att_set_value(scpps_env.shdl + SCPS_IDX_SCAN_REFRESH_NTF_CFG, sizeof(uint16_t),
                                     (uint8_t *)&ntf_cfg);

                if(param->last)
                {
                    // Inform APP of configuration change
                    struct scpps_scan_refresh_ntf_cfg_ind * ind = KE_MSG_ALLOC(SCPPS_SCAN_REFRESH_NTF_CFG_IND,
                                                                               scpps_env.con_info.appid, TASK_SCPPS,
                                                                               scpps_scan_refresh_ntf_cfg_ind);

                    memcpy(&ind->conhdl, &scpps_env.con_info.conhdl, sizeof(uint16_t));
                    memcpy(&ind->scan_refresh_ntf_en, (uint8_t *)&ntf_cfg, sizeof(uint16_t));

                    ke_msg_send(ind);
                }
            }
            else
            {
                status = PRF_APP_ERROR;
            }

            // Send Write Response
            atts_write_rsp_send(scpps_env.con_info.conhdl, param->handle, status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref HTPT_MEAS_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid,
                                       struct gatt_notify_cmp_evt const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
#if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)  
    if(param->status == GATT_NOTIFY_GET_DATA)
        return (KE_MSG_CONSUMED);        
#endif

    scpps_scan_refresh_cfm_send(param->status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to SCPPS.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gap_discon_cmp_evt_handler(ke_msg_id_t const msgid,
                                        struct gap_discon_cmp_evt const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // Check Connection Handle
    if (param->conhdl == scpps_env.con_info.conhdl)
    {
        scpps_disable();
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Disable State handler definition
const struct ke_msg_handler scpps_disabled[] =
{
    {SCPPS_CREATE_DB_REQ,           (ke_msg_func_t) scpps_create_db_req_handler}
};

/// Idle State handler definition.
const struct ke_msg_handler scpps_idle[] =
{
    {SCPPS_ENABLE_REQ,              (ke_msg_func_t) scpps_enable_req_handler}
};

/// Connected State handler definition.
const struct ke_msg_handler scpps_connected[] =
{
    {SCPPS_SCAN_REFRESH_SEND_REQ,   (ke_msg_func_t) scpps_scan_refresh_send_req_handler},
    {GATT_WRITE_CMD_IND,            (ke_msg_func_t) gatt_write_cmd_ind_handler},
    {GATT_NOTIFY_CMP_EVT,           (ke_msg_func_t) gatt_notify_cmp_evt_handler},
};

/// Default State handlers definition
const struct ke_msg_handler scpps_default_state[] =
{
    {GAP_DISCON_CMP_EVT,         (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler scpps_state_handler[SCPPS_STATE_MAX] =
{
    [SCPPS_DISABLED]       = KE_STATE_HANDLER(scpps_disabled),
    [SCPPS_IDLE]           = KE_STATE_HANDLER(scpps_idle),
    [SCPPS_CONNECTED]      = KE_STATE_HANDLER(scpps_connected),
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler scpps_default_handler = KE_STATE_HANDLER(scpps_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t scpps_state[SCPPS_IDX_MAX];

// Registet SCPPS task into kernel
void task_scpps_desc_register(void)
{
    struct ke_task_desc task_scpps_desc;
    
    task_scpps_desc.state_handler = scpps_state_handler;
    task_scpps_desc.default_handler = &scpps_default_handler;
    task_scpps_desc.state = scpps_state;
    task_scpps_desc.state_max = SCPPS_STATE_MAX;
    task_scpps_desc.idx_max = SCPPS_IDX_MAX;
    
    task_desc_register(TASK_SCPPS, task_scpps_desc);
}

#endif /* #if (BLE_SP_SERVER) */

/// @} SCPPSTASK
