/**
 ****************************************************************************************
 *
 * @file app_gatt_task.c
 *
 * @brief Application GATT task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_GATT_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if QN_SVC_DISC_USED

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the Discovery all services complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_SVC_ALL_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_svc_all_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched services from
 *  the remote server. The searched service item include the UUID, start handle and end handle.
 *
 * @note  
 *
 *  GATT service list structure refer to struct gatt_svc_list
 ****************************************************************************************
 */
int app_gatt_disc_svc_all_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_all_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        uint8_t idx = KE_IDX_GET(src_id);
        uint8_t *nb_svc = &app_env.dev_rec[idx].nb_svc;
        for (uint8_t i = 0; i < param->nb_resp && *nb_svc < QN_GATT_MAX_HDL_NB; i++)
        {
            // Profile service should between ATT_SVC_IMMEDIATE_ALERT and ATT_SVC_SCAN_PARAMETERS
            if (param->list[i].attr_hdl >= ATT_SVC_IMMEDIATE_ALERT && param->list[i].attr_hdl <= ATT_SVC_SCAN_PARAMETERS)         
            {
                app_env.dev_rec[idx].svc_rec[*nb_svc].attr_hdl = param->list[i].attr_hdl;
                app_env.dev_rec[idx].nb_svc += 1;
            }
            QPRINTF("UUID: 0x%04X ", param->list[i].attr_hdl);
            QPRINTF("START: 0x%04X ", param->list[i].start_hdl);
            QPRINTF("END: 0x%04X\r\n", param->list[i].end_hdl);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Discovery specific services by UUID complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_SVC_BY_UUID_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_svc_by_uuid_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched services from
 *  the remote server. The searched service item include start handle and end handle.
 *
 * @note  
 *
 *  GATT service list structure refer to struct gatt_svc_range_list
 ****************************************************************************************
 */
int app_gatt_disc_svc_by_uuid_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_by_uuid_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->nb_resp != 0)
    {
        for (uint8_t i = 0; i < param->nb_resp; i++)
        {
            QPRINTF("Service start handle %04X, end handle %04X\r\n", param->list[i].start_hdl, param->list[i].end_hdl);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Discovery included services by UUID complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_SVC_INCL_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_svc_incl_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched services from
 *  the remote server. The searched service item include included service UUID, start handle and end handle.
 *
 ****************************************************************************************
 */
int app_gatt_disc_svc_incl_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_incl_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->nb_entry != 0)
    {
        if (param->entry_len == GATT_UUID_2_OCTET_LEN)
        {
            for (uint8_t i = 0; i < param->nb_entry; i++)
            {
                QPRINTF("Attribute handle 0x%04X, service UUID 0x%04X, start handle 0x%04X, end handle 0x%04X\r\n", param->incl_list.list[i].attr_hdl,
                param->incl_list.list[i].uuid, param->incl_list.list[i].start_hdl, param->incl_list.list[i].end_hdl);
            }
        }
        else // 16 bytes UUID
        {
            // Just include one item if remote device uses 128 bits UUID
            QPRINTF("Attribute handle 0x%04X, service UUID 0x", param->incl_list.incl.attr_hdl);
            for (int8_t i = GATT_UUID_16_OCTET_LEN - 1; i >= 0; i--)
                QPRINTF("%02x", param->incl_list.incl.uuid[i]);
            QPRINTF(", start handle 0x%04X, end handle 0x%04X\r\n", param->incl_list.incl.start_hdl, param->incl_list.incl.end_hdl);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Discovery services with 128 bits UUID complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_SVC_ALL_128_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_svc_all_128_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched services from
 *  the remote server. The searched service item include included service UUID, start handle and end handle.
 *
 ****************************************************************************************
 */
int app_gatt_disc_svc_all_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_svc_all_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Service UUID 0x");
        for (int8_t i = GATT_UUID_16_OCTET_LEN - 1; i >= 0; i--)
            QPRINTF("%02x", param->list.attr_hdl[i]);
        QPRINTF(", start handle 0x%04X, end handle 0x%04X\r\n", param->list.start_hdl, param->list.end_hdl);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the discover all characteristics complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_CHAR_ALL_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_char_all_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched characteristics from
 *  the remote server. The searched characteristics item include properties, pointer handle to
 *  UUID and characteristic UUID.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_all_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_all_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->nb_entry != 0)
    {
        for (uint8_t i = 0; i < param->nb_entry; i++)
        {
            QPRINTF("Attribute handle 0x%04X, properties 0x%02X, value handle 0x%04X, char UUID 0x%04X\r\n", param->list[i].attr_hdl,
                param->list[i].prop, param->list[i].pointer_hdl, param->list[i].uuid);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the discover characteristics by UUID complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_CHAR_BY_UUID_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_char_by_uuid_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched characteristics from
 *  the remote server. The searched characteristics item include the UUID, properties, 
 *  pointer handle to UUID and characteristic UUID.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_by_uuid_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_by_uuid_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->nb_entry != 0)
    {
        for (uint8_t i = 0; i < param->nb_entry; i++)
        {
            QPRINTF("Attribute handle 0x%04X, properties 0x%02X, value handle 0x%04X, char UUID 0x%04X\r\n", param->list[i].attr_hdl,
                param->list[i].prop, param->list[i].pointer_hdl, param->list[i].uuid);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the complete event for discover all 128 bits UUID characteristics. *//**
 *
 * @param[in] msgid     GATT_DISC_CHAR_ALL_128_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_char_all_128_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched characteristics from
 *  the remote server. The searched characteristics item include properties, pointer handle to
 *  UUID and characteristic UUID.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_all_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_all_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Attribute handle 0x%04X, properties 0x%02X, value handle 0x%04X, ", param->list.attr_hdl,
                param->list.prop, param->list.pointer_hdl);
        QPRINTF("char UUID 0x");
        for (int8_t i = GATT_UUID_16_OCTET_LEN - 1; i >= 0; i--)
            QPRINTF("%02x", param->list.uuid[i]);
        QPRINTF("\r\n");
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the complete event for discover specific 128 bits UUID characteristics. *//**
 *
 * @param[in] msgid     GATT_DISC_CHAR_BY_UUID_128_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_char_by_uuid_128_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched characteristics from
 *  the remote server. The searched characteristics item include properties, pointer handle to
 *  UUID and characteristic UUID.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_by_uuid_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_by_uuid_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Attribute handle 0x%04X, properties 0x%02X, value handle 0x%04X, ", param->list.attr_hdl,
                param->list.prop, param->list.pointer_hdl);
        QPRINTF("char UUID 0x");
        for (int8_t i = GATT_UUID_16_OCTET_LEN - 1; i >= 0; i--)
            QPRINTF("%02x", param->list.uuid[i]);
        QPRINTF("\r\n");
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the discover characteristics descriptors complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_CHAR_DESC_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_char_desc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched characteristics 
 *  descriptors from the remote server. The searched characteristics descriptors item include
 *  the database element handle and descriptor UUID.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_desc_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_desc_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->nb_entry != 0)
    {
        for (uint8_t i = 0; i < param->nb_entry; i++)
        {
            QPRINTF("Attribute handle 0x%04X descriptor UUID 0x%04X.\r\n", param->list[i].attr_hdl,
                 param->list[i].desc_hdl);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the complete event for discover all 128 bits UUID characteristics. *//**
 *
 * @param[in] msgid     GATT_DISC_CHAR_ALL_128_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_char_desc_128_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the searched characteristics from
 *  the remote server. The searched characteristics item include properties, pointer handle to
 *  UUID and characteristic UUID.
 *
 ****************************************************************************************
 */
int app_gatt_disc_char_desc_128_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_char_desc_128_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Attribute handle 0x%04X, ", param->list_128.attr_hdl);
    QPRINTF("descriptor UUID 0x");
    for (int8_t i = GATT_UUID_16_OCTET_LEN - 1; i >= 0; i--)
        QPRINTF("%02x", param->list_128.uuid[i]);
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the read characteristic complete event. *//**
 *
 * @param[in] msgid     GATT_READ_CHAR_RESP
 * @param[in] param     Pointer to struct gatt_read_char_resp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the value of the attribute
 *  handle element from the remote server. The element item include the data length and data.
 *
 ****************************************************************************************
 */
int app_gatt_read_char_resp_handler(ke_msg_id_t const msgid, struct gatt_read_char_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Gatt read characteristic sucess, ");
        for (int8_t i = 0; i < param->data.len; i++)
            QPRINTF("%02x", param->data.data[i]);
        QPRINTF("\r\n");
    }
    else
    {
        QPRINTF("Gatt read characteristic failed, status is %d\r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Read multiple characteristic complete event. *//**
 *
 * @param[in] msgid     GATT_READ_CHAR_MULTI_RESP
 * @param[in] param     Pointer to struct gatt_read_char_mult_resp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to receive the multiple value of the 
 *  attribute handle element from the remote server. The element item include
 *  the data length and data.
 *
 ****************************************************************************************
 */
int app_gatt_read_char_mult_resp_handler(ke_msg_id_t const msgid, struct gatt_read_char_mult_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //App should save the set of handles to be read and expected value size when sending read request
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Gatt read multiple characteristic sucess\r\n");
    }
    else
    {
        QPRINTF("Gatt read characteristic failed, status is %d\r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the write characteristic complete event. *//**
 *
 * @param[in] msgid     GATT_WRITE_CHAR_RESP
 * @param[in] param     Pointer to struct gatt_write_char_resp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to handle the result of write characteristics 
 *  value to the remote server.
 *
 ****************************************************************************************
 */
int app_gatt_write_char_resp_handler(ke_msg_id_t const msgid, struct gatt_write_char_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Gatt write sucess\r\n");
    }
    else
    {
        QPRINTF("Gatt write failed, status is %d\r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the write reliable characteristic complete event. *//**
 *
 * @param[in] msgid     GATT_WRITE_CHAR_RELIABLE_RESP
 * @param[in] param     Pointer to struct gatt_write_reliable_resp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to handle the result of write long 
 *  characteristics value to the remote server.
 *
 ****************************************************************************************
 */
int app_gatt_write_char_reliable_resp_handler(ke_msg_id_t const msgid, struct gatt_write_reliable_resp const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        QPRINTF("Gatt reliable write sucess\r\n");
    }
    else
    {
        QPRINTF("Gatt reliable write failed, status is %d\r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the cancel write characteristic complete event. *//**
 *
 * @param[in] msgid     GATT_CANCEL_WRITE_CHAR_RESP
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to handle the result of cancel write
 *  characteristics request.
 *
 ****************************************************************************************
 */
int app_gatt_cancel_write_char_resp_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Gatt cancel write complete\r\n");

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the notify complete event. *//**
 *
 * @param[in] msgid     GATT_NOTIFY_CMP_EVT
 * @param[in] param     Pointer to struct gatt_notify_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to indicate the failed reason of notify
 *  request.
 *
 ****************************************************************************************
 */
int app_gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_notify_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Gatt send notify handle 0x%04x, status %d\r\n", param->handle, param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the value notification. *//**
 *
 * @param[in] msgid     GATT_HANDLE_VALUE_NOTIF
 * @param[in] param     Pointer to struct gatt_handle_value_notif
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to handle the value notification
 *
 ****************************************************************************************
 */
int app_gatt_handle_value_notif_handler(ke_msg_id_t const msgid, struct gatt_handle_value_notif const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Gatt received notify from remote, handle 0x%04x, ", param->charhdl);
    for (uint8_t i = 0; i < param->size; i++)
        QPRINTF("%02x", param->value[i]);
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles reception of of a peer device indication. *//**
 *
 * @param[in] msgid     GATT_HANDLE_VALUE_IND
 * @param[in] param     Pointer to struct gatt_handle_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to handle reception of a peer device
 *  indication.
 *
 ****************************************************************************************
 */
int app_gatt_handle_value_ind_handler(ke_msg_id_t const msgid, struct gatt_handle_value_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Gatt received inication from remote, handle 0x%04x, ", param->charhdl);
    for (uint8_t i = 0; i < param->size; i++)
        QPRINTF("%02x", param->value[i]);
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles reception of a peer device confirmation of a previously sent indication. *//**
 *
 * @param[in] msgid     GATT_HANDLE_VALUE_CFM
 * @param[in] param     Pointer to struct gatt_handle_value_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the Client role of GATT to Handles reception of a peer device
 *  confirmation of a previously sent indication.
 *
 ****************************************************************************************
 */
int app_gatt_handle_value_cfm_handler(ke_msg_id_t const msgid, struct gatt_handle_value_cfm const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Gatt the handle 0x%04x value indication is sent, status %d\r\n", param->handle, param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Discovery complete event. *//**
 *
 * @param[in] msgid     GATT_DISC_CMP_EVT
 * @param[in] param     Pointer to struct gatt_disc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used by the GATT Discovery all services complete event.
 *
 ****************************************************************************************
 */
int app_gatt_disc_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_disc_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Discovery Services finished.\r\n");

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the GATT operation complete event. *//**
 *
 * @param[in] msgid     GATT_CMP_EVT
 * @param[in] param     Pointer to struct gatt_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used to tell application the complete of GATT operations.
 *
 ****************************************************************************************
 */
int app_gatt_cmp_evt_handler(ke_msg_id_t const msgid, struct gatt_cmp_evt const *param,
                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Gatt command ");
    if (param->status == ATT_ERR_NO_ERROR)
        QPRINTF("success.\r\n");
    else
        QPRINTF("failed, status is %d.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_GATT_TASK
