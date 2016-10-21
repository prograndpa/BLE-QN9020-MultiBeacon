/**
 ****************************************************************************************
 *
 * @file blps_task.c
 *
 * @brief Blood Pressure Profile Sensor Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev$
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup BLPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_BP_SENSOR)

#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "blps.h"
#include "blps_task.h"
#include "prf_utils.h"



/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BLPS_CREATE_DB_REQ message.
 * The handler adds BPS into the database using the database
 * configuration value given in param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int blps_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct blps_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    //Service Configuration Flag
    uint16_t cfg_flag = BLPS_MANDATORY_MASK;
    //Database Creation Status
    uint8_t status;

    //Save Application ID
    blps_env.con_info.prf_id = TASK_BLPS;
    //Save Database Configuration
    blps_env.features = param->features;

    // automatically set start handle when creating database
//    blps_env.shdl=0;
    /*---------------------------------------------------*
     * Blood Pressure Service Creation
     *---------------------------------------------------*/
    //Set Configuration Flag Value
    if (BLPS_IS_SUPPORTED(BLPS_INTM_CUFF_PRESS_SUP))
    {
        cfg_flag |= BLPS_INTM_CUFF_PRESS_MASK;
    }

    //Add Service Into Database
    status = atts_svc_create_db(&blps_env.shdl, (uint8_t *)&cfg_flag, BPS_IDX_NB, NULL,
                               dest_id, &blps_att_db[0]);
    //Disable BPS
    attsdb_svc_set_permission(blps_env.shdl, PERM(SVC, DISABLE));


    //Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_BLPS, BLPS_IDLE);
    }

    //Send response to application
    struct blps_create_db_cfm * cfm = KE_MSG_ALLOC(BLPS_CREATE_DB_CFM, src_id,
                                                   TASK_BLPS, blps_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BLPS_ENABLE_REQ message.
 * The handler enables the Blood Pressure Sensor Profile and initialize readable values.
 * @param[in] msgid Id of the message received (probably unused).off
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int blps_enable_req_handler(ke_msg_id_t const msgid,
                                   struct blps_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    //Value used to initialize all readable value in DB
    uint16_t indntf_cfg = 0;

    // Save the application task id
    blps_env.con_info.appid = src_id;
    // Save the connection handle associated to the profile
    blps_env.con_info.conhdl = param->conhdl;

    // Check if the provided connection exist
    if (gap_get_rec_idx(param->conhdl) == GAP_INVALID_CONIDX)
    {
        // The connection doesn't exist, request disallowed
        prf_server_error_ind_send((prf_env_struct *)&blps_env, PRF_ERR_REQ_DISALLOWED,
                                  BLPS_ERROR_IND, BLPS_ENABLE_REQ);
    }
    else
    {
        // Set Blood Pressure Feature Value in database - Not supposed to change during connection
        attsdb_att_set_value(blps_env.shdl + BPS_IDX_BP_FEATURE_VAL, sizeof(uint16_t),
                             (uint8_t *)&param->bp_feature);

        // Configure Blood Pressure Measuremment IND Cfg in DB
        if(param->con_type == PRF_CON_NORMAL)
        {
            memcpy(&indntf_cfg, &param->bp_meas_ind_en, sizeof(uint16_t));

            if (param->bp_meas_ind_en == PRF_CLI_START_IND)
            {
                blps_env.evt_cfg |= BLPS_BP_MEAS_IND_CFG;
            }
        }

        //Set BP Meas. Char. IND Configuration in DB - 0 if not normal connection
        attsdb_att_set_value(blps_env.shdl + BPS_IDX_BP_MEAS_IND_CFG, sizeof(uint16_t),
                             (uint8_t *)&indntf_cfg);

        // Configure Intermediate Cuff Pressure NTF Cfg in DB
        if (BLPS_IS_SUPPORTED(BLPS_INTM_CUFF_PRESS_SUP))
        {
            if(param->con_type == PRF_CON_NORMAL)
            {
                memcpy(&indntf_cfg, &param->interm_cp_ntf_en, sizeof(uint16_t));

                if (param->interm_cp_ntf_en == PRF_CLI_START_NTF)
                {
                    blps_env.evt_cfg |= BLPS_INTM_CUFF_PRESS_NTF_CFG;
                }
            }

            //Set Intm. Cuff Pressure NTF Configuration in DB - 0 if not normal connection
            attsdb_att_set_value(blps_env.shdl + BPS_IDX_INTM_CUFF_PRESS_NTF_CFG, sizeof(uint16_t),
                                 (uint8_t *)&indntf_cfg);
        }

        // Enable Service + Set Security Level
        attsdb_svc_set_permission(blps_env.shdl, param->sec_lvl);

        // Go to connected state
        ke_state_set(TASK_BLPS, BLPS_CONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BLPS_MEAS_SEND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int blps_meas_send_req_handler(ke_msg_id_t const msgid,
                                      struct blps_meas_send_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t packed_bp[BLPS_BP_MEAS_MAX_LEN];
    uint8_t status = PRF_ERR_OK;
    uint8_t size;

    if(param->conhdl == blps_env.con_info.conhdl)
    {
        //Pack the BP Measurement value
        size = blps_pack_meas_value(&packed_bp[0], &param->meas_val);

        //Intermediary blood pressure, must be notified if enabled
        if(param->flag_interm_cp == 0x01)
        {
            //Check if Char. is supported
            if (BLPS_IS_SUPPORTED(BLPS_INTM_CUFF_PRESS_SUP))
            {
                //Check if notifications are enabled
                if(BLPS_IS_ENABLED(BLPS_INTM_CUFF_PRESS_NTF_CFG))
                {
                    //Update value in DB
                    attsdb_att_set_value(blps_env.shdl + BPS_IDX_INTM_CUFF_PRESS_VAL,
                                         size, &packed_bp[0]);

                    //Send notification through GATT
                    struct gatt_notify_req * ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT,
                                                                TASK_BLPS, gatt_notify_req);
                    ntf->conhdl  = blps_env.con_info.conhdl;
                    ntf->charhdl = blps_env.shdl + BPS_IDX_INTM_CUFF_PRESS_VAL;

                    ke_msg_send(ntf);
                }
                //notification not enabled, simply don't send anything
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
        //Stable Blood Pressure Measurement, must be indicated if enabled
        else
        {
            if(BLPS_IS_ENABLED(BLPS_BP_MEAS_IND_CFG))
            {
                //Update value in DB
                attsdb_att_set_value(blps_env.shdl + BPS_IDX_BP_MEAS_VAL,
                                     size, &packed_bp[0]);

                //Send indication through GATT
                struct gatt_indicate_req * ind = KE_MSG_ALLOC(GATT_INDICATE_REQ, TASK_GATT,
                                                              TASK_BLPS, gatt_indicate_req);
                ind->conhdl  = blps_env.con_info.conhdl;
                ind->charhdl = blps_env.shdl + BPS_IDX_BP_MEAS_VAL;

                ke_msg_send(ind);
            }
            else
            {
                status = PRF_ERR_IND_DISABLED;
            }
        }

    }
    else
    {
        //Wrong Connection Handle
        status = (PRF_ERR_INVALID_PARAM);
    }

    // verify that no error occurs
    if(status != PRF_ERR_OK)
    {
        blps_meas_send_cfm_send(status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
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
    uint16_t value = 0x0000;
    uint8_t status = PRF_ERR_OK;
    uint8_t char_code = 0;

    if (param->conhdl == blps_env.con_info.conhdl)
    {
        //Extract value before check
        memcpy(&value, &(param->value), sizeof(uint16_t));

        //BP Measurement Char. - Client Char. Configuration
        if (param->handle == (blps_env.shdl + BPS_IDX_BP_MEAS_IND_CFG))
        {
            if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_IND))
            {
                char_code = BPS_BP_MEAS_CODE;

                if (value == PRF_CLI_STOP_NTFIND)
                {
                    blps_env.evt_cfg &= ~BLPS_BP_MEAS_IND_CFG;
                }
                else //PRF_CLI_START_IND
                {
                    blps_env.evt_cfg |= BLPS_BP_MEAS_IND_CFG;
                }
            }
            else
            {
                status = PRF_APP_ERROR;
            }
        }
        else if (param->handle == (blps_env.shdl + BPS_IDX_INTM_CUFF_PRESS_NTF_CFG))
        {
            if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_NTF))
            {
                char_code = BPS_INTERM_CP_CODE;

                if (value == PRF_CLI_STOP_NTFIND)
                {
                    blps_env.evt_cfg &= ~BLPS_INTM_CUFF_PRESS_NTF_CFG;
                }
                else //PRF_CLI_START_NTF
                {
                    blps_env.evt_cfg |= BLPS_INTM_CUFF_PRESS_NTF_CFG;
                }
            }
            else
            {
                status = PRF_APP_ERROR;
            }
        }

        if (status == PRF_ERR_OK)
        {
            //Update the attribute value
            attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&value);
            if(param->last)
            {
                //Inform APP of configuration change
                struct blps_cfg_indntf_ind * ind = KE_MSG_ALLOC(BLPS_CFG_INDNTF_IND,
                                                                blps_env.con_info.appid, TASK_BLPS,
                                                                blps_cfg_indntf_ind);

                memcpy(&ind->conhdl, &blps_env.con_info.conhdl, sizeof(uint16_t));
                ind->char_code = char_code;
                memcpy(&ind->cfg_val, &value, sizeof(uint16_t));

                ke_msg_send(ind);
            }
        }
    }

    //Send write response
    atts_write_rsp_send(blps_env.con_info.conhdl, param->handle, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_HANDLE_VALUE_CFM message meaning that Measurement indication
 * has been correctly sent to peer device.
 *
 * Convey this information to appli task using @ref BLPS_MEAS_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_handle_value_cfm_handler(ke_msg_id_t const msgid,
                                        struct gatt_handle_value_cfm const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    blps_meas_send_cfm_send(param->status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref BLPS_MEAS_SEND_CFM
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

    blps_meas_send_cfm_send(param->status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to BLPS.
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
    //Check Connection Handle
    if (param->conhdl == blps_env.con_info.conhdl)
    {
        blps_disable();
    }

    return (KE_MSG_CONSUMED);
}


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Disabled State handler definition.
const struct ke_msg_handler blps_disabled[] =
{
    {BLPS_CREATE_DB_REQ,       (ke_msg_func_t) blps_create_db_req_handler}
};

/// Idle State handler definition.
const struct ke_msg_handler blps_idle[] =
{
    {BLPS_ENABLE_REQ,           (ke_msg_func_t) blps_enable_req_handler}
};

/// Connected State handler definition.
const struct ke_msg_handler blps_connected[] =
{
    {GATT_WRITE_CMD_IND,    (ke_msg_func_t) gatt_write_cmd_ind_handler},
    {BLPS_MEAS_SEND_REQ,    (ke_msg_func_t) blps_meas_send_req_handler},
    {GATT_HANDLE_VALUE_CFM, (ke_msg_func_t) gatt_handle_value_cfm_handler},
    {GATT_NOTIFY_CMP_EVT,   (ke_msg_func_t) gatt_notify_cmp_evt_handler},
};

/// Default State handlers definition
const struct ke_msg_handler blps_default_state[] =
{
    {GAP_DISCON_CMP_EVT,         (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler blps_state_handler[BLPS_STATE_MAX] =
{
    [BLPS_DISABLED]       = KE_STATE_HANDLER(blps_disabled),
    [BLPS_IDLE]           = KE_STATE_HANDLER(blps_idle),
    [BLPS_CONNECTED]      = KE_STATE_HANDLER(blps_connected),
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler blps_default_handler = KE_STATE_HANDLER(blps_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t blps_state[BLPS_IDX_MAX];

// Register BLPS task into kernel
void task_blps_desc_register(void)
{
    struct ke_task_desc  task_blps_desc;
    
    task_blps_desc.state_handler = blps_state_handler;
    task_blps_desc.default_handler=&blps_default_handler;
    task_blps_desc.state = blps_state;
    task_blps_desc.state_max = BLPS_STATE_MAX;
    task_blps_desc.idx_max = BLPS_IDX_MAX;

    task_desc_register(TASK_BLPS, task_blps_desc);
}

#endif /* #if (BLE_BP_SENSOR) */

/// @} BLPSTASK
