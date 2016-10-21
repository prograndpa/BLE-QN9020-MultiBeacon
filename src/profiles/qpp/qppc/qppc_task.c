/**
 ****************************************************************************************
 *
 * @file qppc_task.c
 *
 * @brief Quintic Private Profile Client Task implementation.
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
 * @addtogroup QPPCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_QPP_CLIENT)
#include "gap.h"
#include "attm.h"
#include "qppc.h"
#include "qppc_task.h"
#include "gatt_task.h"
#include "smpc_task.h"
#include "qpp_common.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/// State machine used to retrieve Quintic Private service characteristics information
const struct qpp_char_def qppc_qpps_char[2] =
{
    /// Quintic Private Profile Intput Value
    [QPPC_QPPS_RX_CHAR_VALUE]       = {{ATT_UUID_128_LEN, (uint8_t *)QPPS_RX_CHAR_UUID}, ATT_MANDATORY, ATT_CHAR_PROP_WR},
    /// Quintic Private Profile Output Value
    [QPPC_QPPS_FIRST_TX_CHAR_VALUE] = {{ATT_UUID_128_LEN, (uint8_t *)QPPS_FIRST_TX_CHAR_UUID}, ATT_MANDATORY, ATT_CHAR_PROP_NTF},
};

/// State machine used to retrieve Quintic Private service characteristic description information
const struct prf_char_desc_def qppc_qpps_char_desc[2] =
{
    // add user descriptor here
    [QPPC_QPPS_RX_CHAR_VALUE_USER_DESP] = {ATT_DESC_CHAR_USER_DESCRIPTION, ATT_OPTIONAL, QPPC_QPPS_RX_CHAR_VALUE},
    /// Quintic Private Service Output Value client config
    [QPPC_QPPS_FIRST_TX_VALUE_CLI_CFG]  = {ATT_DESC_CLIENT_CHAR_CFG, ATT_MANDATORY, QPPC_QPPS_FIRST_TX_CHAR_VALUE},
};

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
 * @brief Handles reception of the @ref QPPC_ENABLE_REQ message.
 * The handler enables the Quintic Private Profile Client Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qppc_enable_req_handler(ke_msg_id_t const msgid,
                                   struct qppc_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Status
    uint8_t status;
    // QPP Profile Client Role Task Environment
    struct qppc_env_tag *qppc_env;
    // Connection Information
    struct prf_con_info con_info;

    // Save the application task id
    con_info.appid = src_id;
    // Save the profile task id
    con_info.prf_id = dest_id;
    // Save the connection handle associated to the profile
    con_info.conhdl = param->conhdl;

    // Add an environment for the provided device
    status = PRF_CLIENT_ENABLE(con_info, param, qppc);

    if (status == PRF_ERR_OK)
    {
        qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

        //config connection, start discovering
        if(param->con_type == PRF_CON_DISCOVERY)
        {
            prf_disc_qpp_svc_send(&(qppc_env->con_info));
            qppc_env->last_uuid_req = ATT_DECL_PRIMARY_SERVICE;
            // Go to DISCOVERING state
            ke_state_set(dest_id, QPPC_DISCOVERING);
        }
        //normal connection, get saved att details
        else
        {
            qppc_env->qpps = param->qpps;
            qppc_enable_cfm_send(qppc_env, &con_info, status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_SVC_BY_UUID_CMP_EVT message.
 * The handler stores the found service details for service discovery.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_svc_by_uuid_evt_handler(ke_msg_id_t const msgid,
                                             struct gatt_disc_svc_by_uuid_cmp_evt const *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        //even if we get multiple responses we only store 1 range
        qppc_env->qpps.svc.shdl = param->list[0].start_hdl;
        qppc_env->qpps.svc.ehdl = param->list[0].end_hdl;
        qppc_env->nb_svc++;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_CHAR_ALL_CMP_EVT message.
 * Characteristics for the currently desired service handle range are obtained and kept.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_char_all_128_evt_handler(ke_msg_id_t const msgid,
                                          struct gatt_disc_char_all_128_cmp_evt const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        //Save properties and handles
        qppc_env->qpps.chars[qppc_env->nb_char].char_hdl = param->list.attr_hdl;
        qppc_env->qpps.chars[qppc_env->nb_char].val_hdl = param->list.pointer_hdl;
        qppc_env->qpps.chars[qppc_env->nb_char].prop = param->list.prop;
        qppc_env->qpps.chars[qppc_env->nb_char].char_ehdl_off = 3;
        qppc_env->nb_char++;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_CHAR_DESC_CMP_EVT message.
 * This event can be received several times
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_char_desc_evt_handler(ke_msg_id_t const msgid,
                                           struct gatt_disc_char_desc_cmp_evt const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    if (qppc_env->last_uuid_req == ATT_DESC_CHAR_USER_DESCRIPTION)
    {
        qppc_env->qpps.descs[qppc_env->last_char_code].desc_hdl = param->list[0].attr_hdl;
    }
    else if (qppc_env->last_uuid_req == ATT_DESC_CLIENT_CHAR_CFG)
    {
        //Retrieve characteristic descriptor handle using UUID
        for (uint8_t i = 0; i < (param->nb_entry) && qppc_env->last_char_code < qppc_env->nb_char; i++)
        {
            if (param->list[i].desc_hdl == ATT_DESC_CLIENT_CHAR_CFG)
            {
                qppc_env->qpps.descs[qppc_env->last_char_code].desc_hdl = param->list[i].attr_hdl;
            }
        }
    }
    
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_CMP_EVT message.
 * This generic event is received for different requests, so need to keep track.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gatt_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    uint8_t status = 0;
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    if ((param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND)||
        (param->status == ATT_ERR_NO_ERROR))
    {
        // service start/end handles has been received
        if(qppc_env->last_uuid_req == ATT_DECL_PRIMARY_SERVICE)
        {
            // check if service handles are not ok
            if(qppc_env->qpps.svc.shdl == ATT_INVALID_HANDLE)
            {
                // stop discovery procedure.
                qppc_enable_cfm_send(qppc_env, &qppc_env->con_info, PRF_ERR_STOP_DISC_CHAR_MISSING);
            }
            //too many services found only one such service should exist
            else if(qppc_env->nb_svc != 1)
            {
                // stop discovery procedure.
                qppc_enable_cfm_send(qppc_env, &qppc_env->con_info, PRF_ERR_MULTIPLE_SVC);
            }
            else
            {
                //discover all QPPS characteristics
                prf_disc_char_all_send(&(qppc_env->con_info), &(qppc_env->qpps.svc));
                qppc_env->nb_char = 0;
                qppc_env->last_uuid_req = ATT_DECL_CHARACTERISTIC;
            }
        }
        else if(qppc_env->last_uuid_req == ATT_DECL_CHARACTERISTIC)
        {
            status = prf_check_svc_128_char_validity(2, qppc_env->qpps.chars,
                                                 qppc_qpps_char);

            // Check for characteristic properties.
            if(status == PRF_ERR_OK)
            {
                qppc_env->last_uuid_req = ATT_DESC_CHAR_USER_DESCRIPTION;
                qppc_env->last_char_code = qppc_qpps_char_desc[QPPC_QPPS_RX_CHAR_VALUE_USER_DESP].char_code;

                //Discover Quintic Private profile Configuration Descriptor - Mandatory
                prf_disc_char_desc_send(&(qppc_env->con_info),
                                        &(qppc_env->qpps.chars[qppc_env->last_char_code]));
            }
            else
            {
                // Stop discovery procedure.
                qppc_enable_cfm_send(qppc_env, &qppc_env->con_info, status);
            }
        }
        else if (qppc_env->last_uuid_req == ATT_DESC_CHAR_USER_DESCRIPTION)
        {
            qppc_env->last_uuid_req = ATT_DESC_CLIENT_CHAR_CFG;
            qppc_env->last_char_code = qppc_qpps_char_desc[QPPC_QPPS_FIRST_TX_VALUE_CLI_CFG].char_code;
            //Discover Quintic Private profile User Descriptor
            prf_disc_char_desc_send(&(qppc_env->con_info),
                                    &(qppc_env->qpps.chars[qppc_env->last_char_code]));
        }
        else if (qppc_env->last_uuid_req == ATT_DESC_CLIENT_CHAR_CFG)
        {
            if (qppc_env->last_char_code < qppc_env->nb_char)
            {
                prf_disc_char_desc_send(&(qppc_env->con_info),
                                        &(qppc_env->qpps.chars[++qppc_env->last_char_code]));
            }
            else
            {
                status = prf_check_svc_char_desc_validity(2,
                                                          qppc_env->qpps.descs,
                                                          qppc_qpps_char_desc,
                                                          qppc_env->qpps.chars);
                qppc_enable_cfm_send(qppc_env, &qppc_env->con_info, status);
            }
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPC_RD_CHAR_REQ message.
 * Check if the handle exists in profile(already discovered) and send request, otherwise
 * error to APP.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qppc_rd_char_req_handler(ke_msg_id_t const msgid,
                                    struct qppc_rd_char_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    uint16_t search_hdl = ATT_INVALID_SEARCH_HANDLE;
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    if (param->conhdl == qppc_env->con_info.conhdl && 
        param->char_code < qppc_env->nb_char)
    {
        qppc_env->last_char_code = param->char_code;
        search_hdl = qppc_env->qpps.descs[param->char_code].desc_hdl;

        //check if handle is variable
        if (search_hdl != ATT_INVALID_SEARCH_HANDLE)
        {
            prf_read_char_send(&(qppc_env->con_info), qppc_env->qpps.svc.shdl, qppc_env->qpps.svc.ehdl, search_hdl);
        }
        else
        {
            qppc_error_ind_send(qppc_env, PRF_ERR_INEXISTENT_HDL);
        }
    }
    else
    {
        qppc_error_ind_send(qppc_env, PRF_ERR_INVALID_PARAM);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_READ_CHAR_RESP message.
 * Generic event received after every simple read command sent to peer server.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_rd_char_rsp_handler(ke_msg_id_t const msgid,
                                    struct gatt_read_char_resp const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    struct qppc_rd_char_rsp * rsp = KE_MSG_ALLOC_DYN(QPPC_RD_CHAR_RSP,
                                                     qppc_env->con_info.appid, dest_id,
                                                     qppc_rd_char_rsp, param->data.len);

    rsp->conhdl = qppc_env->con_info.conhdl;
    rsp->status = param->status;
    rsp->char_code = qppc_env->last_char_code;
    rsp->data.len = param->data.len;
    rsp->data.each_len = param->data.each_len;
    memcpy(&rsp->data.data[0], &param->data.data[0], param->data.len);

    ke_msg_send(rsp);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPC_CFG_INDNTF_REQ message.
 * It allows configuration of the peer ind/ntf/stop characteristic for a specified characteristic.
 * Will return an error code if that cfg char does not exist.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qppc_cfg_indntf_req_handler(ke_msg_id_t const msgid,
                                       struct qppc_cfg_indntf_req const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t status = PRF_ERR_OK;
    uint16_t cfg_hdl = 0x0000;

    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    do {
        if(param->conhdl != qppc_env->con_info.conhdl)
        {
            status = PRF_ERR_INVALID_PARAM;
            break;
        }

        if(!(param->char_code < qppc_env->nb_char))
        {
            status = PRF_ERR_INVALID_PARAM;
            break;
        }

        if(!((param->cfg_val == PRF_CLI_STOP_NTFIND)||
                (param->cfg_val == PRF_CLI_START_NTF)))
        {
            status = PRF_ERR_INVALID_PARAM;
            break;
        }

        //get handle of the configuration characteristic to set and check if value matches property
        cfg_hdl = qppc_env->qpps.descs[param->char_code].desc_hdl;
        if (cfg_hdl == ATT_INVALID_SEARCH_HANDLE)
        {
            status = PRF_ERR_INEXISTENT_HDL;
            break;
        }

        qppc_env->last_char_code = param->char_code;
        // Send GATT Write Request
        prf_gatt_write_ntf_ind(&qppc_env->con_info, cfg_hdl, param->cfg_val);
    } while(0);

    if (status != PRF_ERR_OK)
    {
        qppc_error_ind_send(qppc_env, status);
    }

    return (KE_MSG_CONSUMED);
}

/*
****************************************************************************************
* @brief Handles reception of the @ref QPPC_WR_DATA_REQ message.
* Check if the handle exists in profile(already discovered) and send request, otherwise
* error to APP.
* @param[in] msgid Id of the message received (probably unused).
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance (probably unused).
* @param[in] src_id ID of the sending task instance.
* @return If the message was consumed or not.
****************************************************************************************
*/
static int qppc_wr_data_req_handler(ke_msg_id_t const msgid,
                                    struct qppc_wr_data_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

   if(param->conhdl == qppc_env->con_info.conhdl)
   {
       //this is mandatory readable if it is included in the peer's DB
       if (qppc_env->qpps.chars[QPPC_QPPS_RX_CHAR_VALUE].char_hdl != ATT_INVALID_SEARCH_HANDLE)
       {
           if ((qppc_env->qpps.chars[QPPC_QPPS_RX_CHAR_VALUE].prop & ATT_CHAR_PROP_WR_NO_RESP) == ATT_CHAR_PROP_WR_NO_RESP)
           {
               // Send GATT Write Request, simple write, not no response
               prf_gatt_write(&qppc_env->con_info, qppc_env->qpps.chars[QPPC_QPPS_RX_CHAR_VALUE].val_hdl,
                              (uint8_t *)param->data, CO_MIN(param->length, QPP_DATA_MAX_LEN), GATT_WRITE_NO_RESPONSE);
           }
           //write not allowed, so no point in continuing
           else
           {
               qppc_error_ind_send(qppc_env, PRF_ERR_NOT_WRITABLE);
           }
       }
       //send app error indication for inexistent handle for this characteristic
       else
       {
           qppc_error_ind_send(qppc_env, PRF_ERR_INEXISTENT_HDL);
       }
   }
   else
   {
       qppc_error_ind_send(qppc_env, PRF_ERR_INVALID_PARAM);
   }
   return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CHAR_RESP message.
 * Generic event received after every simple write command sent to peer server.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_char_rsp_handler(ke_msg_id_t const msgid,
                                       struct gatt_write_char_resp const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    struct qppc_wr_char_rsp *wr_cfm = KE_MSG_ALLOC(QPPC_WR_CHAR_RSP,
                                                   qppc_env->con_info.appid, dest_id,
                                                   qppc_wr_char_rsp);
    wr_cfm->conhdl = qppc_env->con_info.conhdl;
    wr_cfm->char_code = qppc_env->last_char_code;
    //it will be a GATT status code
    wr_cfm->status = param->status;
    // send the message
    ke_msg_send(wr_cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_HANDLE_VALUE_NTF message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_handle_value_ntf_handler(ke_msg_id_t const msgid,
                                        struct gatt_handle_value_notif const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    uint8_t char_code = 0;
    // Get the address of the environment
    struct qppc_env_tag *qppc_env = PRF_CLIENT_GET_ENV(dest_id, qppc);

    if (param->conhdl == qppc_env->con_info.conhdl)
    {
        char_code = (param->charhdl - qppc_env->qpps.chars[QPPC_QPPS_FIRST_TX_CHAR_VALUE].val_hdl) / 3 + 1;

        if (qppc_env->qpps.chars[char_code].val_hdl == param->charhdl)
        {
            struct qppc_data_ind * ind = KE_MSG_ALLOC_DYN(QPPC_DATA_IND,
                                                          qppc_env->con_info.appid, dest_id,
                                                          qppc_data_ind, param->size);
            // retrieve connection handle
            ind->conhdl = qppc_env->con_info.conhdl;
            ind->char_code = char_code;
            ind->length = param->size;
            memcpy(ind->data, param->value, ind->length);

            ke_msg_send(ind);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to QPPC.
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
    PRF_CLIENT_DISABLE_IND_SEND(qppc_envs, dest_id, QPPC);
    
    // message is consumed
    return (KE_MSG_CONSUMED);
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
// IDLE State handlers definition.
const struct ke_msg_handler qppc_idle[] =
{
    {QPPC_ENABLE_REQ,        (ke_msg_func_t)qppc_enable_req_handler},
};

// Specifies the message handlers for the connected state
const struct ke_msg_handler qppc_connected[] =
{
    {QPPC_RD_CHAR_REQ,       (ke_msg_func_t)qppc_rd_char_req_handler},
    {GATT_READ_CHAR_RESP,    (ke_msg_func_t)gatt_rd_char_rsp_handler},
    {QPPC_CFG_INDNTF_REQ,    (ke_msg_func_t)qppc_cfg_indntf_req_handler},
    {QPPC_WR_DATA_REQ,       (ke_msg_func_t)qppc_wr_data_req_handler},
    {GATT_WRITE_CHAR_RESP,   (ke_msg_func_t)gatt_write_char_rsp_handler},
    {GATT_HANDLE_VALUE_NOTIF,(ke_msg_func_t)gatt_handle_value_ntf_handler},
};

/// Specifies the Discovering state message handlers
const struct ke_msg_handler qppc_discovering[] =
{
    {GATT_DISC_SVC_BY_UUID_CMP_EVT, (ke_msg_func_t)gatt_disc_svc_by_uuid_evt_handler},
    {GATT_DISC_CHAR_ALL_128_CMP_EVT,    (ke_msg_func_t)gatt_disc_char_all_128_evt_handler},
    {GATT_DISC_CHAR_DESC_CMP_EVT,   (ke_msg_func_t)gatt_disc_char_desc_evt_handler},
    {GATT_CMP_EVT,                  (ke_msg_func_t)gatt_cmp_evt_handler},
};

/// Default State handlers definition
const struct ke_msg_handler qppc_default_state[] =
{
    {GAP_DISCON_CMP_EVT,     (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

// Specifies the message handler structure for every input state.
const struct ke_state_handler qppc_state_handler[QPPC_STATE_MAX] =
{
    [QPPC_IDLE]        = KE_STATE_HANDLER(qppc_idle),
    [QPPC_CONNECTED]   = KE_STATE_HANDLER(qppc_connected),
    [QPPC_DISCOVERING] = KE_STATE_HANDLER(qppc_discovering),
};

// Specifies the message handlers that are common to all states.
const struct ke_state_handler qppc_default_handler = KE_STATE_HANDLER(qppc_default_state);

// Defines the place holder for the states of all the task instances.
ke_state_t qppc_state[QPPC_IDX_MAX];

// Register QPPC task into kernel 
void task_qppc_desc_register(void)
{
    struct ke_task_desc  task_qppc_desc;
    
    task_qppc_desc.state_handler = qppc_state_handler;
    task_qppc_desc.default_handler = &qppc_default_handler;
    task_qppc_desc.state = qppc_state;
    task_qppc_desc.state_max = QPPC_STATE_MAX;
    task_qppc_desc.idx_max = QPPC_IDX_MAX;

    task_desc_register(TASK_QPPC, task_qppc_desc);
}

#endif /* (BLE_QPP_CLIENT) */
/// @} QPPCTASK
