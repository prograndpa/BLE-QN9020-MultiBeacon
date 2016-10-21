/**
 ****************************************************************************************
 *
 * @file hrps_task.c
 *
 * @brief Heart Rate Profile Sensor Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup HRPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_HR_SENSOR)
#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "hrps.h"
#include "hrps_task.h"

#include "prf_utils.h"


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HRPS_CREATE_DB_REQ message.
 * The handler adds HRS into the database using the database
 * configuration value given in param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hrps_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct hrps_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    //Service Configuration Flag
    uint8_t cfg_flag = HRPS_MANDATORY_MASK;
    //Database Creation Status
    uint8_t status;

    //Save Profile ID
    hrps_env.con_info.prf_id = TASK_HRPS;
    //Save Database Configuration
    hrps_env.features = param->features;

    /*---------------------------------------------------*
     * Heart Rate Service Creation
     *---------------------------------------------------*/
    //Set Configuration Flag Value
    if (HRPS_IS_SUPPORTED(HRPS_BODY_SENSOR_LOC_CHAR_SUP))
    {
        cfg_flag |= HRPS_BODY_SENSOR_LOC_MASK;
    }
    if (HRPS_IS_SUPPORTED(HRPS_ENGY_EXP_FEAT_SUP))
    {
        cfg_flag |= HRPS_HR_CTNL_PT_MASK;
    }

    //Add Service Into Database
    status = atts_svc_create_db(&hrps_env.shdl, (uint8_t *)&cfg_flag, HRS_IDX_NB, NULL,
                               dest_id, &hrps_att_db[0]);
    //Disable HRS
    attsdb_svc_set_permission(hrps_env.shdl, PERM(SVC, DISABLE));

    //Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_HRPS, HRPS_IDLE);
    }

    //Send response to application
    struct hrps_create_db_cfm * cfm = KE_MSG_ALLOC(HRPS_CREATE_DB_CFM, src_id,
                                                   TASK_HRPS, hrps_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HRPS_ENABLE_REQ message.
 * The handler enables the Heart Rate Sensor Profile.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hrps_enable_req_handler(ke_msg_id_t const msgid,
                                   struct hrps_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t value = 0;

    // Save the application task id
    hrps_env.con_info.appid = src_id;
    // Save the connection handle associated to the profile
    hrps_env.con_info.conhdl = param->conhdl;

    // Check if the provided connection exist
    if (gap_get_rec_idx(param->conhdl) == GAP_INVALID_CONIDX)
    {
        // The connection doesn't exist, request disallowed
        prf_server_error_ind_send((prf_env_struct *)&hrps_env, PRF_ERR_REQ_DISALLOWED,
                                  HRPS_ERROR_IND, HRPS_ENABLE_REQ);
    }
    else
    {
        // If this connection is a not configuration one, apply config saved by app
        if(param->con_type == PRF_CON_NORMAL)
        {
            value = param->hr_meas_ntf_en;

            if (param->hr_meas_ntf_en == PRF_CLI_START_NTF)
            {
                hrps_env.features |= HRPS_HR_MEAS_NTF_CFG;
            }
        }

        //Set HR Meas. Char. NTF Configuration in DB
        attsdb_att_set_value(hrps_env.shdl + HRS_IDX_HR_MEAS_NTF_CFG, sizeof(uint16_t),
                             (uint8_t *)&value);

        if (HRPS_IS_SUPPORTED(HRPS_BODY_SENSOR_LOC_CHAR_SUP))
        {
            //Set Body Sensor Location Char Value in DB - Not supposed to change during connection
            attsdb_att_set_value(hrps_env.shdl + HRS_IDX_BOBY_SENSOR_LOC_VAL, sizeof(uint8_t),
                                 (uint8_t *)&param->body_sensor_loc);
        }

        // Enable Service + Set Security Level
        attsdb_svc_set_permission(hrps_env.shdl, param->sec_lvl);

        // Go to connected state
        ke_state_set(TASK_HRPS, HRPS_CONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HRPS_MEAS_SEND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hrps_meas_send_req_handler(ke_msg_id_t const msgid,
                                      struct hrps_meas_send_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Packed Heart Measurement Value
    uint8_t packed_hr[HRPS_HT_MEAS_MAX_LEN];
    // Packet size
    uint8_t size;

    if((param->conhdl == hrps_env.con_info.conhdl)
            && (param->meas_val.nb_rr_interval <= HRS_MAX_RR_INTERVAL))
    {
        //Pack the HR Measurement value
        size = hrps_pack_meas_value(&packed_hr[0], &param->meas_val);

        // Check if notifications are enabled
        if(HRPS_IS_SUPPORTED(HRPS_HR_MEAS_NTF_CFG))
        {
            //Update value in DB
            attsdb_att_set_value(hrps_env.shdl + HRS_IDX_HR_MEAS_VAL,
                                 size, &packed_hr[0]);

            //send notification through GATT
            struct gatt_notify_req * ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT,
                                                        TASK_HRPS, gatt_notify_req);
            ntf->conhdl  = hrps_env.con_info.conhdl;
            ntf->charhdl = hrps_env.shdl + HRS_IDX_HR_MEAS_VAL;

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
        status = PRF_ERR_INVALID_PARAM;
    }

    if (status != PRF_ERR_OK)
    {
        // Value has not been sent
        hrps_meas_send_cfm_send(status);
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

    if (param->conhdl == hrps_env.con_info.conhdl)
    {
        //BP Measurement Char. - Client Char. Configuration
        if (param->handle == (hrps_env.shdl + HRS_IDX_HR_MEAS_NTF_CFG))
        {
            //Extract value before check
            memcpy(&value, &(param->value), sizeof(uint16_t));

            if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_NTF))
            {
                if (value == PRF_CLI_STOP_NTFIND)
                {
                    hrps_env.features &= ~HRPS_HR_MEAS_NTF_CFG;
                }
                else //PRF_CLI_START_NTF
                {
                    hrps_env.features |= HRPS_HR_MEAS_NTF_CFG;
                }
            }
            else
            {
                status = PRF_APP_ERROR;
            }

            if (status == PRF_ERR_OK)
            {
                //Update the attribute value
                attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&value);
                if(param->last)
                {
                    //Inform APP of configuration change
                    struct hrps_cfg_indntf_ind * ind = KE_MSG_ALLOC(HRPS_CFG_INDNTF_IND,
                                                                    hrps_env.con_info.appid, TASK_HRPS,
                                                                    hrps_cfg_indntf_ind);

                    memcpy(&ind->conhdl, &hrps_env.con_info.conhdl, sizeof(uint16_t));
                    memcpy(&ind->cfg_val, &value, sizeof(uint16_t));

                    ke_msg_send(ind);
                }
            }
        }
        //HR Control Point Char. Value
        else
        {
            if (HRPS_IS_SUPPORTED(HRPS_ENGY_EXP_FEAT_SUP))
            {
                //Extract value
                memcpy(&value, &(param->value), sizeof(uint8_t));

                if (value == 0x1)
                {
                    //inform APP of configuration change
                    struct hrps_energy_exp_reset_ind * ind = KE_MSG_ALLOC(HRPS_ENERGY_EXP_RESET_IND,
                                                                          hrps_env.con_info.appid,
                                                                          TASK_HRPS,
                                                                          hrps_energy_exp_reset_ind);

                    memcpy(&ind->conhdl, &(hrps_env.con_info.conhdl), sizeof(uint16_t));

                    ke_msg_send(ind);
                }
                else
                {
                    status = HRS_ERR_HR_CNTL_POINT_NOT_SUPPORTED;
                }
            }
            else
            {
                //Allowed to send Application Error if value inconvenient
                status = HRS_ERR_HR_CNTL_POINT_NOT_SUPPORTED;
            }
        }
    }

    //Send write response
    atts_write_rsp_send(hrps_env.con_info.conhdl, param->handle, status);

    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref HRPS_MEAS_SEND_CFM
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

    hrps_meas_send_cfm_send(param->status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to HRPS.
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
    if (param->conhdl == hrps_env.con_info.conhdl)
    {
        hrps_disable();
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Disabled State handler definition.
const struct ke_msg_handler hrps_disabled[] =
{
    {HRPS_CREATE_DB_REQ,       (ke_msg_func_t) hrps_create_db_req_handler}
};

/// Idle State handler definition.
const struct ke_msg_handler hrps_idle[] =
{
    {HRPS_ENABLE_REQ,       (ke_msg_func_t) hrps_enable_req_handler}
};

/// Connected State handler definition.
const struct ke_msg_handler hrps_connected[] =
{
    {HRPS_MEAS_SEND_REQ,    (ke_msg_func_t) hrps_meas_send_req_handler},
    {GATT_WRITE_CMD_IND,    (ke_msg_func_t) gatt_write_cmd_ind_handler},
    {GATT_NOTIFY_CMP_EVT,   (ke_msg_func_t) gatt_notify_cmp_evt_handler},
};

/* Default State handlers definition. */
const struct ke_msg_handler hrps_default_state[] =
{
    {GAP_DISCON_CMP_EVT,         (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler hrps_state_handler[HRPS_STATE_MAX] =
{
    [HRPS_DISABLED]       = KE_STATE_HANDLER(hrps_disabled),
    [HRPS_IDLE]           = KE_STATE_HANDLER(hrps_idle),
    [HRPS_CONNECTED]      = KE_STATE_HANDLER(hrps_connected),
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler hrps_default_handler = KE_STATE_HANDLER(hrps_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t hrps_state[HRPS_IDX_MAX];

// Register HRPS task into kernel
void task_hrps_desc_register(void)
{
    struct ke_task_desc  task_hrps_desc;
    
    task_hrps_desc.state_handler = hrps_state_handler;
    task_hrps_desc.default_handler=&hrps_default_handler;
    task_hrps_desc.state = hrps_state;
    task_hrps_desc.state_max = HRPS_STATE_MAX;
    task_hrps_desc.idx_max = HRPS_IDX_MAX;

    task_desc_register(TASK_HRPS, task_hrps_desc);
}

#endif /* #if (BLE_HR_SENSOR) */

/// @} HRPSTASK
