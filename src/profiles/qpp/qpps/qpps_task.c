/**
 ****************************************************************************************
 *
 * @file qpps_task.c
 *
 * @brief Quintic Private Profile Server Task Implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup QPPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_QPP_SERVER)
#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "qpps.h"
#include "qpps_task.h"
#include "ke_mem.h"
#include "prf_utils.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPS_CREATE_DB_REQ message.
 * The handler adds Quintic private service into the database using the database
 * configuration value given in param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 //warning: initialiing 'uint8_t *' (aka 'unsigned char *') with an expression of type 'char[17] converts between pointers to integer types with different sign

uint8_t *qpp_uuid_list[] = 
{
    (uint8_t *)"\x01\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
    (uint8_t *)"\x02\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
    (uint8_t *)"\x03\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
    (uint8_t *)"\x04\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
    (uint8_t *)"\x05\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
    (uint8_t *)"\x06\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
    (uint8_t *)"\x07\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4",
};

static int qpps_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct qpps_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    //Service Configuration Flag
    uint64_t cfg_flag = QPPS_MANDATORY_MASK;
    //Database Creation Status
    uint8_t status;
    uint8_t idx_nb = 4;
    uint8_t tx_char_num = param->tx_char_num;
    struct atts_desc_ext *qpps_db = NULL;
    struct atts_char128_desc *char_desc_def = NULL;

    if (tx_char_num > 7)
        tx_char_num = 7;

    //Save Application ID
    qpps_env.appid = src_id;
    qpps_env.ntf_char_num = tx_char_num;
    /*---------------------------------------------------*
     * Quintic private Service Creation
     *---------------------------------------------------*/

    if (tx_char_num > 1)
    {
        qpps_db = (struct atts_desc_ext *)ke_malloc(sizeof(qpps_att_db) + 3 * (tx_char_num) * sizeof(struct atts_desc_ext));
        char_desc_def = (struct atts_char128_desc *)ke_malloc((tx_char_num) * sizeof(struct atts_char128_desc));
        for (uint8_t i = 0; i < tx_char_num; i++)
        {
            struct atts_char128_desc value_char = ATTS_CHAR128(ATT_CHAR_PROP_NTF,
                                                                0,
                                                            QPPS_FIRST_TX_CHAR_UUID);
            
            value_char.attr_type[0] += i;
            char_desc_def[i] = value_char;
        }

        memcpy(qpps_db, qpps_att_db, sizeof(qpps_att_db));

        for (uint8_t i = 0; i < tx_char_num; i++)
        {
            qpps_db[QPPS_IDX_VAL_CHAR + i * 3] = qpps_db[QPPS_IDX_VAL_CHAR];
            qpps_db[QPPS_IDX_VAL_CHAR + i * 3].value = (uint8_t*)&char_desc_def[i];
            qpps_db[QPPS_IDX_VAL_CHAR + i * 3 + 1] = qpps_db[QPPS_IDX_VAL];
            qpps_db[QPPS_IDX_VAL_CHAR + i * 3 + 1].type.uuid = qpp_uuid_list[i];
            qpps_db[QPPS_IDX_VAL_CHAR + i * 3 + 2] = qpps_db[QPPS_IDX_VAL + 1];
        }
    }

    while (tx_char_num--)
    {
        cfg_flag = (cfg_flag << 3) | 0x07;
        idx_nb += 3;
    }
    //Add Service Into Database
    status = atts_svc_create_db_ext(&qpps_env.shdl, (uint8_t *)&cfg_flag, idx_nb, NULL,
                               dest_id, (param->tx_char_num <= 1 ? &qpps_att_db[0] : &qpps_db[0]));

    if (qpps_db != NULL)
        ke_free(qpps_db);
    if (char_desc_def != NULL)
        ke_free(char_desc_def);
    
    //Disable QPPS
    attsdb_svc_set_permission(qpps_env.shdl, PERM(SVC, DISABLE));

    //Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_QPPS, QPPS_IDLE);
    }

    //Send response to application
    struct qpps_create_db_cfm * cfm = KE_MSG_ALLOC(QPPS_CREATE_DB_CFM, qpps_env.appid,
                                                   TASK_QPPS, qpps_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPS_ENABLE_REQ message.
 * The handler enables the Quintic private Profile.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qpps_enable_req_handler(ke_msg_id_t const msgid,
                                   struct qpps_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t value = 0;

    // Save the application task id
    qpps_env.appid = src_id;
    // Save the connection handle associated to the profile
    qpps_env.conhdl = param->conhdl;

    // If this connection is a not configuration one, apply config saved by app
    if(param->con_type == PRF_CON_NORMAL)
    {
        qpps_env.features = param->ntf_en;
        for (uint8_t i = 0; i < qpps_env.ntf_char_num; i++)
        {
            //Set Val NTF Configuration in DB
              if (QPPS_IS_SUPPORTED(i, QPPS_VALUE_NTF_CFG))
            {
                value = QPPS_VALUE_NTF_CFG;
                attsdb_att_set_value(qpps_env.shdl + QPPS_IDX_VAL_NTF_CFG + i*3, sizeof(uint16_t),
                                    (uint8_t *)&value);
            }
            else
            {
                value = 0;
                attsdb_att_set_value(qpps_env.shdl + QPPS_IDX_VAL_NTF_CFG + i*3, sizeof(uint16_t),
                                    (uint8_t *)&value);
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < qpps_env.ntf_char_num; i++)
        {
            //Set Val NTF Configuration in DB
            attsdb_att_set_value(qpps_env.shdl + QPPS_IDX_VAL_NTF_CFG + i*3, sizeof(uint16_t),
                                 (uint8_t *)&value);
        }
    }

    // Enable Service + Set Security Level
    attsdb_svc_set_permission(qpps_env.shdl, param->sec_lvl);

    // Go to connected state
    ke_state_set(TASK_QPPS, QPPS_CONNECTED);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPS_DATA_SEND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qpps_data_send_req_handler(ke_msg_id_t const msgid,
                                      struct qpps_data_send_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if((param->conhdl == qpps_env.conhdl) &&
       (param->length <= QPP_DATA_MAX_LEN) &&
       (param->index <= qpps_env.ntf_char_num))
    {
        // Check if notifications are enabled
        if(QPPS_IS_SUPPORTED(param->index, QPPS_VALUE_NTF_CFG))
        {
            //Update value in DB
            attsdb_att_set_value(qpps_env.shdl + QPPS_IDX_VAL + param->index * 3,
                                 param->length, (void *)param->data);

            //send notification through GATT
            struct gatt_notify_req * ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT,
                                                        TASK_QPPS, gatt_notify_req);
            ntf->conhdl  = qpps_env.conhdl;
            ntf->charhdl = qpps_env.shdl + QPPS_IDX_VAL + param->index * 3;

            ke_msg_send(ntf);
        }
        //notification not enabled, simply don't send anything
        else
        {
            // Send CFM to APP that value was not sent not
            struct qpps_data_send_cfm * cfm = KE_MSG_ALLOC(QPPS_DATA_SEND_CFM, qpps_env.appid,
                    TASK_QPPS, qpps_data_send_cfm);

            cfm->conhdl = qpps_env.conhdl;
            cfm->status = PRF_ERR_NTF_DISABLED;;

            ke_msg_send(cfm);
        }
    }
    else
    {
        // Wrong Connection Handle
        qpps_error_ind_send(PRF_ERR_INVALID_PARAM);
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
    uint8_t status = PRF_ERR_OK;

    if (param->conhdl == qpps_env.conhdl)
    {
        // Client Char. Configuration
        uint8_t char_index = param->handle - (qpps_env.shdl + QPPS_IDX_VAL_NTF_CFG);
        if ((param->handle > (qpps_env.shdl + QPPS_IDX_VAL_CHAR)) && ((char_index % 3) == 0))
        {
            uint16_t value = 0x0000;

            //Extract value before check
            memcpy(&value, &(param->value), sizeof(uint16_t));

            if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_NTF))
            {
                if (value == PRF_CLI_STOP_NTFIND)
                {
                    qpps_env.features &= ~(QPPS_VALUE_NTF_CFG << (char_index / 3));
                }
                else //PRF_CLI_START_NTF
                {
                    qpps_env.features |= QPPS_VALUE_NTF_CFG << (char_index / 3);
                }
            }
            else
            {
                status = PRF_APP_ERROR;
            }

            if (status == PRF_ERR_OK)
            {
                uint8_t *old_value;
                atts_size_t length;

                attsdb_att_get_value(param->handle, &length, &old_value);
                if (value != co_read16p(old_value))
                {
                    //Update the attribute value
                    attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&value);

                    if(param->last)
                    {
                        //Inform APP of configuration change
                        struct qpps_cfg_indntf_ind * ind = KE_MSG_ALLOC(QPPS_CFG_INDNTF_IND,
                                                                        qpps_env.appid, TASK_QPPS,
                                                                        qpps_cfg_indntf_ind);

                        ind->char_index = (char_index / 3);
                        memcpy(&ind->cfg_val, &value, sizeof(uint16_t));

                        ke_msg_send(ind);
                    }
                }
            }
        }
        else if (param->handle == (qpps_env.shdl + QPPS_IDX_RX_DATA_VAL))
        {
            if (param->length <= QPP_DATA_MAX_LEN)
            {
                //inform APP of configuration change
                struct qpps_data_val_ind * ind = KE_MSG_ALLOC_DYN(QPPS_DAVA_VAL_IND,
                                                                  qpps_env.appid,
                                                                  TASK_QPPS,
                                                                  qpps_data_val_ind, param->length);

                memcpy(&ind->conhdl, &(qpps_env.conhdl), sizeof(uint16_t));
                //Send received data to app value
                ind->length = param->length;
                memcpy(ind->data, param->value, param->length);

                ke_msg_send(ind);
            }
            else
            {
                status = QPPS_ERR_RX_DATA_EXCEED_MAX_LENGTH;
            }
        }
        else
        {
            status = QPPS_ERR_INVALID_PARAM;
        }
    }

    if (param->response)
    {
        //Send write response
        atts_write_rsp_send(qpps_env.conhdl, param->handle, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref QPPS_MEAS_SEND_CFM
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
    // Send CFM to APP that value was not sent not
    struct qpps_data_send_cfm * cfm = KE_MSG_ALLOC(QPPS_DATA_SEND_CFM, qpps_env.appid,
                                                   TASK_QPPS, qpps_data_send_cfm);

    cfm->conhdl = qpps_env.conhdl;
    cfm->char_index = (param->handle - (qpps_env.shdl + QPPS_IDX_VAL)) / 3;
    cfm->status = param->status;

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Disconnection indication to QPPS.
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
    if (param->conhdl == qpps_env.conhdl)
    {
        qpps_disable();
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Disabled State handler definition.
const struct ke_msg_handler qpps_disabled[] =
{
    {QPPS_CREATE_DB_REQ,        (ke_msg_func_t) qpps_create_db_req_handler}
};

/// Idle State handler definition.
const struct ke_msg_handler qpps_idle[] =
{
    {QPPS_ENABLE_REQ,           (ke_msg_func_t) qpps_enable_req_handler}
};

/// Connected State handler definition.
const struct ke_msg_handler qpps_connected[] =
{
    {QPPS_DATA_SEND_REQ,    (ke_msg_func_t) qpps_data_send_req_handler},
    {GATT_WRITE_CMD_IND,    (ke_msg_func_t) gatt_write_cmd_ind_handler},
    {GATT_NOTIFY_CMP_EVT,   (ke_msg_func_t) gatt_notify_cmp_evt_handler},
};

/* Default State handlers definition. */
const struct ke_msg_handler qpps_default_state[] =
{
    {GAP_DISCON_CMP_EVT,    (ke_msg_func_t) gap_discon_cmp_evt_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler qpps_state_handler[QPPS_STATE_MAX] =
{
    [QPPS_DISABLED]       = KE_STATE_HANDLER(qpps_disabled),
    [QPPS_IDLE]           = KE_STATE_HANDLER(qpps_idle),
    [QPPS_CONNECTED]      = KE_STATE_HANDLER(qpps_connected),
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler qpps_default_handler = KE_STATE_HANDLER(qpps_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t qpps_state[QPPS_IDX_MAX];

// Register QPPS task into kernel
void task_qpps_desc_register(void)
{
    struct ke_task_desc  task_qpps_desc;
    
    task_qpps_desc.state_handler = qpps_state_handler;
    task_qpps_desc.default_handler=&qpps_default_handler;
    task_qpps_desc.state = qpps_state;
    task_qpps_desc.state_max = QPPS_STATE_MAX;
    task_qpps_desc.idx_max = QPPS_IDX_MAX;

    task_desc_register(TASK_QPPS, task_qpps_desc);
}

#endif /* #if (BLE_QPP_SERVER) */

/// @} QPPSTASK
