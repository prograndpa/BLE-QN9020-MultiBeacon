/**
 ****************************************************************************************
 *
 * @file hogpbh_task.c
 *
 * @brief HID Over GATT Profile Boot Host Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev$
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HOGPBHTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_HID_BOOT_HOST)

#include "gap.h"
#include "attm.h"
#include "hogpbh.h"
#include "hogpbh_task.h"
#include "gatt_task.h"
#include "hogp_common.h"

/*
 * LOCAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/// State machine used to retrieve HID Service characteristics information
const struct prf_char_def hogpbh_hids_char[HOGPBH_CHAR_MAX] =
{
    /// Protocol Mode
    [HOGPBH_CHAR_PROTO_MODE]             = {ATT_CHAR_PROTOCOL_MODE,
                                            ATT_MANDATORY,
                                            ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR_NO_RESP},
    /// Boot Keyboard Input Report
    [HOGPBH_CHAR_BOOT_KB_IN_REPORT]      = {ATT_CHAR_BOOT_KB_IN_REPORT,
                                            ATT_OPTIONAL,
                                            ATT_CHAR_PROP_RD | ATT_CHAR_PROP_NTF},
    /// Boot Keyboard Output Report
    [HOGPBH_CHAR_BOOT_KB_OUT_REPORT]     = {ATT_CHAR_BOOT_KB_OUT_REPORT,
                                            ATT_OPTIONAL,
                                            ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR},
    /// Boot Keyboard Output Report
    [HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT]   = {ATT_CHAR_BOOT_MOUSE_IN_REPORT,
                                            ATT_OPTIONAL,
                                            ATT_CHAR_PROP_RD | ATT_CHAR_PROP_NTF},
};

/// State machine used to retrieve HID Service characteristic description information
const struct prf_char_desc_def hogpbh_hids_char_desc[HOGPBH_DESC_MAX] =
{
    /// Boot Keyboard Input Report Client Config
    [HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG]    = {ATT_DESC_CLIENT_CHAR_CFG, ATT_OPTIONAL, HOGPBH_CHAR_BOOT_KB_IN_REPORT},
    /// Boot Mouse Input Report Client Config
    [HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG] = {ATT_DESC_CLIENT_CHAR_CFG, ATT_OPTIONAL, HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT},
};

/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_ENABLE_REQ message.
 * The handler enables the HID Over GATT Profile Boot Host Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hogpbh_enable_req_handler(ke_msg_id_t const msgid,
                                   struct hogpbh_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Status
    uint8_t status;
    // HID Over GATT Profile Client Role Task Environment
    struct hogpbh_env_tag *hogpbh_env;
    // Connection Information
    struct prf_con_info con_info;

    // Fill the Connection Information structure
    con_info.conhdl = param->conhdl;
    con_info.prf_id = dest_id;
    con_info.appid  = src_id;

    // Add an environment for the provided device
    status =  PRF_CLIENT_ENABLE(con_info, param, hogpbh);

    if (status == PRF_ERR_FEATURE_NOT_SUPPORTED)
    {
        // The message has been forwarded to another task id.
        return (KE_MSG_NO_FREE);
    }
    else if (status == PRF_ERR_OK)
    {
        hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

        //config connection, start discovering
        if(param->con_type == PRF_CON_DISCOVERY)
        {
            //start discovering HIDS on peer
            prf_disc_svc_send(&(hogpbh_env->con_info), ATT_SVC_HID);

            hogpbh_env->last_uuid_req = ATT_SVC_HID;

            // Go to DISCOVERING state
            ke_state_set(dest_id, HOGPBH_DISCOVERING);
        }
        //normal connection, get saved att details
        else
        {
            memcpy(&hogpbh_env->hids[0], &param->hids[0], HOGPBH_NB_HIDS_INST_MAX*sizeof(struct hids_content));
            hogpbh_env->hids_nb = param->hids_nb;

            //send APP confirmation that can start normal connection to TH
            hogpbh_enable_cfm_send(hogpbh_env, &con_info, PRF_ERR_OK);
        }
    }
    else
    {
        hogpbh_enable_cfm_send(NULL, &con_info, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_DISABLE_REQ message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hogpbh_disable_req_handler(ke_msg_id_t const msgid,
                                      struct hogpbh_disable_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);
    // Counter
    uint8_t hids_nb;

    for (hids_nb = 0; hids_nb < hogpbh_env->hids_nb; hids_nb++)
    {
        // Unregister the profile in the GATT
        prf_unregister_atthdl2gatt(&hogpbh_env->con_info, &hogpbh_env->hids[hids_nb].svc);
    }

    PRF_CLIENT_DISABLE_IND_SEND(hogpbh_envs, dest_id, HOGPBH);

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_RD_CHAR_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hogpbh_rd_char_req_handler(ke_msg_id_t const msgid,
                                      struct hogpbh_rd_char_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Attribute handle
    uint16_t search_hdl = ATT_INVALID_SEARCH_HANDLE;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    // Save the attribute read code
    hogpbh_env->last_char_code = param->char_code;

    if((param->conhdl == hogpbh_env->con_info.conhdl) &&
       (param->hids_nb < HOGPBH_NB_HIDS_INST_MAX))
    {
        // Descriptor
        if(((param->char_code & HOGPBH_DESC_MASK) == HOGPBH_DESC_MASK) &&
           ((param->char_code & ~HOGPBH_DESC_MASK) < HOGPBH_DESC_MASK))
        {
            search_hdl = hogpbh_env->hids[param->hids_nb].descs[param->char_code & ~HOGPBH_DESC_MASK].desc_hdl;
        }
        // Characteristic
        else if (param->char_code < HOGPBH_CHAR_MAX)
        {
            search_hdl = hogpbh_env->hids[param->hids_nb].chars[param->char_code].val_hdl;
        }

        // Check if handle is viable
        if (search_hdl != ATT_INVALID_SEARCH_HANDLE)
        {
            prf_read_char_send(&(hogpbh_env->con_info), hogpbh_env->hids[param->hids_nb].svc.shdl,
                               hogpbh_env->hids[param->hids_nb].svc.ehdl, search_hdl);

            // Save the service instance number
            hogpbh_env->last_svc_inst_req  = param->hids_nb;
        }
        else
        {
            status = PRF_ERR_INEXISTENT_HDL;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;

    }

    if (status != PRF_ERR_OK)
    {
        hogpbh_char_req_rsp_send(hogpbh_env, HOGPBH_RD_CHAR_ERR_RSP, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_CFG_INDNTF_REQ message.
 * It allows configuration of the peer ntf/stop characteristic for a specified characteristic.
 * Will return an error code if that cfg char does not exist.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hogpbh_cfg_ntf_req_handler(ke_msg_id_t const msgid,
                                         struct hogpbh_cfg_ntf_req const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    // Descriptor handle
    uint16_t cfg_hdl;
    // Status
    uint8_t status = PRF_ERR_INVALID_PARAM;
    // Descriptor Code
    uint8_t desc_code = param->desc_code & ~HOGPBH_DESC_MASK;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    // Save the attribute read code
    hogpbh_env->last_char_code = param->desc_code;

    if((param->conhdl == hogpbh_env->con_info.conhdl) &&
       (param->hids_nb < HOGPBH_NB_HIDS_INST_MAX) &&
       (desc_code < HOGPBH_DESC_MAX))
    {
        // Check value to write
        if((param->ntf_cfg == PRF_CLI_STOP_NTFIND) || (param->ntf_cfg == PRF_CLI_START_NTF))
        {
            cfg_hdl = hogpbh_env->hids[param->hids_nb].descs[desc_code].desc_hdl;

            // Check if the handle value exists
            if (cfg_hdl != ATT_INVALID_SEARCH_HANDLE)
            {
                // Send GATT Write Request
                prf_gatt_write_ntf_ind(&hogpbh_env->con_info, cfg_hdl, param->ntf_cfg);

                status = PRF_ERR_OK;
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }
    }

    if (status != PRF_ERR_OK)
    {
        hogpbh_char_req_rsp_send(hogpbh_env, HOGPBH_WR_CHAR_RSP, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_BOOT_KB_OUT_REPORT_WR_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hogpbh_boot_report_wr_req_handler(ke_msg_id_t const msgid,
                                                  struct hogpbh_boot_report_wr_req const *param,
                                                  ke_task_id_t const dest_id,
                                                  ke_task_id_t const src_id)
{
    // Descriptor handle
    uint16_t val_hdl = ATT_INVALID_SEARCH_HANDLE;
    // Status
    uint8_t status = PRF_ERR_OK;
    // Write Type
    uint8_t wr_type = GATT_WRITE_CHAR;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    // Save the Characteristic Code
    hogpbh_env->last_char_code = param->char_code;

    // Check Connection Handle and HIDS Instance Nb
    if((param->conhdl == hogpbh_env->con_info.conhdl) &&
       (param->hids_nb < hogpbh_env->hids_nb) &&
       (param->report_length <= HOGPBH_BOOT_REPORT_MAX_LEN))
    {
        switch (param->char_code)
        {
            case (HOGPBH_CHAR_BOOT_KB_OUT_REPORT):

                // Write Without Response is available for the Boot Keyboard Ouput Report
                if (param->wr_type == GATT_WRITE_NO_RESPONSE)
                {
                    wr_type = param->wr_type;
                }

                break;

            case (HOGPBH_CHAR_BOOT_KB_IN_REPORT):
            case (HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT):

                // Check if the characteristic is writable
                if ((hogpbh_env->hids[param->hids_nb].chars[param->char_code].prop & ATT_CHAR_PROP_WR)
                                                                                  != ATT_CHAR_PROP_WR)
                {
                    status = PRF_ERR_NOT_WRITABLE;
                }

                break;

            default:
                status = PRF_ERR_INVALID_PARAM;
                break;
        }

        if (status == PRF_ERR_OK)
        {
            val_hdl = hogpbh_env->hids[param->hids_nb].chars[param->char_code].val_hdl;

            // Check if the handle value exists
            if (val_hdl != ATT_INVALID_SEARCH_HANDLE)
            {
                prf_gatt_write(&hogpbh_env->con_info, val_hdl, (uint8_t *)&param->report[0],
                               param->report_length, wr_type);
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }

    if (status != PRF_ERR_OK)
    {
        hogpbh_char_req_rsp_send(hogpbh_env, HOGPBH_WR_CHAR_RSP, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HOGPBH_SET_BOOT_PROTO_MODE_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hogpbh_set_boot_proto_mode_req_handler(ke_msg_id_t const msgid,
                                                  struct hogpbh_set_boot_proto_mode_req const *param,
                                                  ke_task_id_t const dest_id,
                                                  ke_task_id_t const src_id)
{
    // Descriptor handle
    uint16_t val_hdl;
    // Status
    uint8_t status = PRF_ERR_OK;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);
    // Protocol Mode
    uint8_t proto_mode = HOGP_BOOT_PROTOCOL_MODE;

    // Save the Characteristic Code
    hogpbh_env->last_char_code = HOGPBH_RD_WR_HIDS_PROTO_MODE;

    // Check Connection Handle and HIDS Instance Nb
    if((param->conhdl == hogpbh_env->con_info.conhdl) &&
       (param->hids_nb < HOGPBH_NB_HIDS_INST_MAX))
    {
        val_hdl = hogpbh_env->hids[param->hids_nb].chars[HOGPBH_CHAR_PROTO_MODE].val_hdl;

        // Check if the handle value exists
        if (val_hdl != ATT_INVALID_SEARCH_HANDLE)
        {
            // Send GATT Write Request
            prf_gatt_write(&hogpbh_env->con_info, val_hdl,
                           &proto_mode, sizeof(uint8_t), GATT_WRITE_NO_RESPONSE);
        }
        else
        {
            status = PRF_ERR_INEXISTENT_HDL;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }

    if (status != PRF_ERR_OK)
    {
        hogpbh_char_req_rsp_send(hogpbh_env, HOGPBH_WR_CHAR_RSP, status);
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
    // Counter
    uint8_t i = 0x00;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    if (param->status == PRF_ERR_OK)
    {
        // We can have several instances of HID Service
        for (i = 0; i < (param->nb_resp); i++)
        {
            if (hogpbh_env->hids_nb < HOGPBH_NB_HIDS_INST_MAX)
            {
                hogpbh_env->hids[hogpbh_env->hids_nb].svc.shdl = param->list[i].start_hdl;
                hogpbh_env->hids[hogpbh_env->hids_nb].svc.ehdl = param->list[i].end_hdl;
            }

            // Update number of found HIDS instances
            hogpbh_env->hids_nb++;
        }
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
static int gatt_disc_char_all_evt_handler(ke_msg_id_t const msgid,
                                          struct gatt_disc_char_all_cmp_evt const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    if (param->status == PRF_ERR_OK)
    {
        // Retrieve HIDS characteristics
        prf_search_chars(hogpbh_env->hids[hogpbh_env->last_svc_inst_req].svc.ehdl, HOGPBH_CHAR_MAX,
                         &hogpbh_env->hids[hogpbh_env->last_svc_inst_req].chars[0], &hogpbh_hids_char[0],
                         param, &hogpbh_env->last_char_code);
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
    //Counters
    uint8_t i;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    //Retrieve characteristic descriptor handle using UUID
    for(i = 0; i<(param->nb_entry); i++)
    {
        if (param->list[i].desc_hdl == ATT_DESC_CLIENT_CHAR_CFG)
        {
            if (hogpbh_env->last_char_code == HOGPBH_CHAR_BOOT_KB_IN_REPORT)
            {
                if ((hogpbh_env->hids[hogpbh_env->last_svc_inst_req].chars[HOGPBH_CHAR_BOOT_KB_IN_REPORT].char_hdl + 2) == param->list[i].attr_hdl)
                {
                    hogpbh_env->hids[hogpbh_env->last_svc_inst_req].descs[HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG].desc_hdl = param->list[i].attr_hdl;
                }
            }
            else if (hogpbh_env->last_char_code == HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT)
            {
                if ((hogpbh_env->hids[hogpbh_env->last_svc_inst_req].chars[HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT].char_hdl + 2) == param->list[i].attr_hdl)
                {
                    hogpbh_env->hids[hogpbh_env->last_svc_inst_req].descs[HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG].desc_hdl = param->list[i].attr_hdl;
                }
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
    // Status
    uint8_t status;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    if ((param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND) ||
        (param->status == ATT_ERR_NO_ERROR))
    {
        /*-------------------------------------------------------------------------------------------------------------
         * SERVICES DISCOVERY
         *-------------------------------------------------------------------------------------------------------------*/
        if (hogpbh_env->last_uuid_req == ATT_SVC_HID)
        {
            // If no HIDS instance has been found
            if (hogpbh_env->hids_nb == 0)
            {
                // Stop discovery procedure.
                hogpbh_enable_cfm_send(hogpbh_env, &hogpbh_env->con_info, PRF_ERR_STOP_DISC_CHAR_MISSING);
            }
            else
            {
                // Discover all HOGPBH characteristics for first service instance
                hogpbh_env->last_uuid_req      = ATT_DECL_CHARACTERISTIC;
                hogpbh_env->last_svc_inst_req  = 0;

                prf_disc_char_all_send(&(hogpbh_env->con_info), &(hogpbh_env->hids[0].svc));
            }
        }

        /*-------------------------------------------------------------------------------------------------------------
         * CHARACTERISTICS DISCOVERY
         *-------------------------------------------------------------------------------------------------------------*/
        else if (hogpbh_env->last_uuid_req == ATT_DECL_CHARACTERISTIC)
        {
            status = prf_check_svc_char_validity(HOGPBH_CHAR_MAX, hogpbh_env->hids[hogpbh_env->last_svc_inst_req].chars,
                                                 hogpbh_hids_char);

            if (status == PRF_ERR_OK)
            {
                hogpbh_env->last_uuid_req  = ATT_INVALID_HANDLE;

                // Check if Boot Keyboard Input Report Char has been found
                if (HOGPBH_FOUND_CHAR(hogpbh_env, HOGPBH_CHAR_BOOT_KB_IN_REPORT))
                {
                    hogpbh_env->last_char_code = HOGPBH_CHAR_BOOT_KB_IN_REPORT;
                }
                // Check if Boot Mouse Input Report Char has been found
                else if (HOGPBH_FOUND_CHAR(hogpbh_env, HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT))
                {
                    hogpbh_env->last_char_code = HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT;
                }
                // Error - At least one of these characteristic must be supported
                else
                {
                    status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                }
            }

            if (status != PRF_ERR_OK)
            {
                //If we are here, stop discovery
                hogpbh_enable_cfm_send(hogpbh_env, &hogpbh_env->con_info, status);
            }
            else
            {
                // Send discover descriptors request
                prf_disc_char_desc_send(&(hogpbh_env->con_info),
                                        &(hogpbh_env->hids[hogpbh_env->last_svc_inst_req].chars[hogpbh_env->last_char_code]));
            }
        }

        /*-------------------------------------------------------------------------------------------------------------
         * DESCRIPTORS DISCOVERY
         *-------------------------------------------------------------------------------------------------------------*/
        else
        {
            if (hogpbh_env->last_char_code == HOGPBH_CHAR_BOOT_KB_IN_REPORT)
            {
                if (HOGPBH_FOUND_CHAR(hogpbh_env, HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT))
                {
                    hogpbh_env->last_char_code = HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT;

                    // Discover Client Char. Cfg descriptor for Boot Mouse Input Report Char.
                    prf_disc_char_desc_send(&(hogpbh_env->con_info),
                                            &(hogpbh_env->hids[hogpbh_env->last_svc_inst_req].chars[hogpbh_env->last_char_code]));
                }
                else
                {
                    if (hogpbh_env->last_svc_inst_req < (hogpbh_env->hids_nb - 1))
                    {
                        // Discover HIDS Characteristics for the next instance
                        hogpbh_env->last_uuid_req = ATT_DECL_CHARACTERISTIC;
                        hogpbh_env->last_svc_inst_req++;

                        prf_disc_char_all_send(&(hogpbh_env->con_info), &(hogpbh_env->hids[hogpbh_env->last_svc_inst_req].svc));
                    }
                    else
                    {
                        // send app the details about the discovered BAPS DB to save
                        hogpbh_enable_cfm_send(hogpbh_env, &hogpbh_env->con_info, PRF_ERR_OK);
                    }
                }
            }

            else if (hogpbh_env->last_char_code == HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT)
            {
                if (hogpbh_env->last_svc_inst_req < (hogpbh_env->hids_nb - 1))
                {
                    // Discover HIDS Characteristics for the next instance
                    hogpbh_env->last_uuid_req = ATT_DECL_CHARACTERISTIC;
                    hogpbh_env->last_svc_inst_req++;

                    prf_disc_char_all_send(&(hogpbh_env->con_info), &(hogpbh_env->hids[hogpbh_env->last_svc_inst_req].svc));
                }
                else
                {
                    // send app the details about the discovered BAPS DB to save
                    hogpbh_enable_cfm_send(hogpbh_env, &hogpbh_env->con_info, PRF_ERR_OK);
                }
            }
        }
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
    uint8_t error_status = param->status;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    if (error_status == PRF_ERR_OK)
    {
        switch (hogpbh_env->last_char_code)
        {
            // Boot Keyboard Input Report Read Response
            case (HOGPBH_CHAR_BOOT_KB_IN_REPORT):
            // Boot Mouse Input Report Read Response
            case (HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT):
            // Boot Mouse Input Report Read Response
            case (HOGPBH_CHAR_BOOT_KB_OUT_REPORT):
            {
                // Check report length
                if (param->data.len <= HOGPBH_BOOT_REPORT_MAX_LEN)
                {
                    struct hogpbh_boot_report_ind *ind = KE_MSG_ALLOC_DYN(HOGPBH_BOOT_REPORT_IND,
                                                                          hogpbh_env->con_info.appid, dest_id,
                                                                          hogpbh_boot_report_ind,
                                                                          param->data.len);

                    ind->conhdl         = hogpbh_env->con_info.conhdl;
                    ind->ind_type       = HOGPBH_IND_RD_RSP;
                    ind->hids_nb        = hogpbh_env->last_svc_inst_req;
                    ind->char_code      = hogpbh_env->last_char_code;
                    ind->report_length  = param->data.len;
                    memcpy(&ind->report[0], &param->data.data[0], param->data.len);

                    ke_msg_send(ind);
                }
                else
                {
                    error_status = PRF_ERR_UNEXPECTED_LEN;
                }

                break;
            }

            //Protocol Mode Read Response
            case (HOGPBH_CHAR_PROTO_MODE):
            {
                struct hogpbh_proto_mode_rd_rsp *rsp = KE_MSG_ALLOC(HOGPBH_PROTO_MODE_RD_RSP,
                                                                    hogpbh_env->con_info.appid, dest_id,
                                                                    hogpbh_proto_mode_rd_rsp);

                rsp->conhdl     = hogpbh_env->con_info.conhdl;
                rsp->hids_nb    = hogpbh_env->last_svc_inst_req;
                memcpy(&rsp->proto_mode, &param->data.data[0], sizeof(uint8_t));

                ke_msg_send(rsp);

                break;
            }

            //Client Characteristic Configuration
            case (HOGPBH_RD_WR_HIDS_BOOT_KB_IN_REPORT_CFG):
            case (HOGPBH_RD_WR_HIDS_BOOT_MOUSE_IN_REPORT_CFG):
            {
                struct hogpbh_cfg_ntf_rd_rsp *rsp = KE_MSG_ALLOC(HOGPBH_CFG_NTF_RD_RSP,
                                                                 hogpbh_env->con_info.appid, dest_id,
                                                                 hogpbh_cfg_ntf_rd_rsp);

                rsp->conhdl     = hogpbh_env->con_info.conhdl;
                rsp->desc_code  = hogpbh_env->last_char_code;
                rsp->hids_nb    = hogpbh_env->last_svc_inst_req;
                memcpy(&rsp->cfg_val, &param->data.data[0], sizeof(uint16_t));

                ke_msg_send(rsp);

                break;
            }

            default:
                break;
        }
    }

    if (error_status != PRF_ERR_OK)
    {
        hogpbh_char_req_rsp_send(hogpbh_env, HOGPBH_RD_CHAR_ERR_RSP, error_status);
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
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    // Send the write char. rsp to the application
    hogpbh_char_req_rsp_send(hogpbh_env, HOGPBH_WR_CHAR_RSP, param->status);

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
    // HIDS Instance
    uint8_t hids_nb;
    // Characteristic Code
    uint8_t char_code = PRF_APP_ERROR;
    // Get the address of the environment
    struct hogpbh_env_tag *hogpbh_env = PRF_CLIENT_GET_ENV(dest_id, hogpbh);

    if (param->conhdl == hogpbh_env->con_info.conhdl)
    {
        // HIDS instance is unknown.
        for (hids_nb = 0; ((hids_nb < hogpbh_env->hids_nb) && (char_code == PRF_APP_ERROR)); hids_nb++)
        {
            //Boot Input Report
            if (param->charhdl == hogpbh_env->hids[hids_nb].chars[HOGPBH_CHAR_BOOT_KB_IN_REPORT].val_hdl)
            {
                char_code = HOGPBH_CHAR_BOOT_KB_IN_REPORT;
            }
            else if (param->charhdl == hogpbh_env->hids[hids_nb].chars[HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT].val_hdl)
            {
                char_code = HOGPBH_CHAR_BOOT_MOUSE_IN_REPORT;
            }

            if (char_code != PRF_APP_ERROR)
            {
                // Check Report size
                if (param->size <= HOGPBH_BOOT_REPORT_MAX_LEN)
                {
                    struct hogpbh_boot_report_ind *ind = KE_MSG_ALLOC_DYN(HOGPBH_BOOT_REPORT_IND,
                                                                          hogpbh_env->con_info.appid, dest_id,
                                                                          hogpbh_boot_report_ind,
                                                                          param->size);

                    ind->conhdl = param->conhdl;
                    ind->ind_type = HOGPBH_IND_NTF;
                    ind->report_length = param->size;
                    ind->hids_nb = hids_nb;
                    ind->char_code = char_code;
                    memcpy(&ind->report[0], &param->value[0], param->size);

                    ke_msg_send(ind);
                }
            }
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to HOGPBH.
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
    PRF_CLIENT_DISABLE_IND_SEND(hogpbh_envs, dest_id, HOGPBH);

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Specifies the message handlers for the connected state
const struct ke_msg_handler hogpbh_connected[] =
{
    {HOGPBH_DISABLE_REQ,                    (ke_msg_func_t)hogpbh_disable_req_handler},
    {HOGPBH_RD_CHAR_REQ,                    (ke_msg_func_t)hogpbh_rd_char_req_handler},
    {HOGPBH_CFG_NTF_REQ,                    (ke_msg_func_t)hogpbh_cfg_ntf_req_handler},
    {HOGPBH_SET_BOOT_PROTO_MODE_REQ,        (ke_msg_func_t)hogpbh_set_boot_proto_mode_req_handler},
    {HOGPBH_BOOT_REPORT_WR_REQ,             (ke_msg_func_t)hogpbh_boot_report_wr_req_handler},
    {GATT_READ_CHAR_RESP,                   (ke_msg_func_t)gatt_rd_char_rsp_handler},
    {GATT_WRITE_CHAR_RESP,                  (ke_msg_func_t)gatt_write_char_rsp_handler},
    {GATT_HANDLE_VALUE_NOTIF,               (ke_msg_func_t)gatt_handle_value_ntf_handler},
};

/// Specifies the Discovering state message handlers
const struct ke_msg_handler hogpbh_discovering[] =
{
    {GATT_DISC_SVC_BY_UUID_CMP_EVT,         (ke_msg_func_t)gatt_disc_svc_by_uuid_evt_handler},
    {GATT_DISC_CHAR_ALL_CMP_EVT,            (ke_msg_func_t)gatt_disc_char_all_evt_handler},
    {GATT_DISC_CHAR_DESC_CMP_EVT,           (ke_msg_func_t)gatt_disc_char_desc_evt_handler},
    {GATT_CMP_EVT,                          (ke_msg_func_t)gatt_cmp_evt_handler},
};

/// Default State handlers definition
const struct ke_msg_handler hogpbh_default_state[] =
{
    {HOGPBH_ENABLE_REQ,                     (ke_msg_func_t)hogpbh_enable_req_handler},
    {GAP_DISCON_CMP_EVT,                    (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

// Specifies the message handler structure for every input state.
const struct ke_state_handler hogpbh_state_handler[HOGPBH_STATE_MAX] =
{
    [HOGPBH_IDLE]        = KE_STATE_HANDLER_NONE,
    [HOGPBH_CONNECTED]   = KE_STATE_HANDLER(hogpbh_connected),
    [HOGPBH_DISCOVERING] = KE_STATE_HANDLER(hogpbh_discovering),
};

// Specifies the message handlers that are common to all states.
const struct ke_state_handler hogpbh_default_handler = KE_STATE_HANDLER(hogpbh_default_state);

// Defines the place holder for the states of all the task instances.
ke_state_t hogpbh_state[HOGPBH_IDX_MAX];

// Register HOGPBH task into kernel
void task_hogpbh_desc_register(void)
{
    struct ke_task_desc task_hogpbh_desc;
    
    task_hogpbh_desc.state_handler = hogpbh_state_handler;
    task_hogpbh_desc.default_handler = &hogpbh_default_handler;
    task_hogpbh_desc.state = hogpbh_state;
    task_hogpbh_desc.state_max = HOGPBH_STATE_MAX;
    task_hogpbh_desc.idx_max = HOGPBH_IDX_MAX;

    task_desc_register(TASK_HOGPBH, task_hogpbh_desc);
}

#endif /* (BLE_HOG_BOOT_HOST) */
/// @} HOGPBHTASK
