/**
 ****************************************************************************************
 *
 * @file app_gatt.c
 *
 * @brief Application GATT API
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
 * @addtogroup APP_GATT_API
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
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Gatt Discovery Service request. *//**
 * @param[in] req_type      GATT request type:
 * - GATT_DISC_ALL_SVC       
 * - GATT_DISC_BY_UUID_SVC   
 * - GATT_DISC_INCLUDED_SVC  
 * @param[in] conhdl        Connection handle.
 * @response  GATT_DISC_SVC_ALL_CMP_EVT or GATT_DISC_SVC_BY_UUID_CMP_EVT or GATT_DISC_SVC_INCL_CMP_EVT
 * and GATT_DISC_CMP_EVT
 * @description
 *
 *  This API is used by the application to send a GATT_DISC_SVC_REQ with the parameters 
 *  deduced from the req_type and desired_svc. The definition for the different codes 
 *  for req_type can be found in gatt.h. Upon reception of this message, GATT will
 *  checks whether the parameters are correct, if not correct then the GATT_DISC_CMP_EVT
 *  message will be generically built and sent to Application directly. An error status
 *  is also possible be either GATT_INVALID_PARAM_ERR or GATT_INVALID_TYPE_IN_SVC_SEARCH.
 *  If parameter is correct the GATT_DISC_SVC_ALL_CMP_EVT message will be received with 
 *  the searched UUID, start handle and end handle together. Once all serviced be got, 
 *  in the second case, the GATT_DISC_CMP_EVT message is sent to Application.
 *
 ****************************************************************************************
 */
void app_gatt_disc_svc_req(uint8_t req_type, uint16_t conhdl)
{   
    struct gatt_disc_svc_req *msg = KE_MSG_ALLOC(GATT_DISC_SVC_REQ, TASK_GATT, TASK_APP,
                                                gatt_disc_svc_req);
    //Connection handle
    msg->conhdl = conhdl;
    //GATT request type
    msg->req_type = req_type;
    //Start handle range
    msg->start_hdl = 0x0001;
    //End handle range
    msg->end_hdl = GATT_MAX_ATTR_HDL;
    //Desired service: if 0x0000, discover all
    msg->desired_svc.value_size = GATT_UUID_2_OCTET_LEN;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Characteristic discovery request. *//**
 * @param[in] req_type      GATT request type:
 * - GATT_DISC_ALL_CHAR
 * - GATT_DISC_BY_UUID_CHAR
 * - GATT_DISC_DESC_CHAR
 * @param[in] conhdl        Connection handle.
 * @response  GATT_DISC_CHAR_ALL_CMP_EVT or GATT_DISC_CHAR_BY_UUID_CMP_EVT or GATT_CMP_EVT
 * @description
 *
 *  This API is used by the application to send a GATT_DISC_CHAR_REQ with the parameters 
 *  deduced from the req_type. The definition for the different codes for req_type can be
 *  found in gatt.h. Upon reception of this message, GATT will checks whether the parameters
 *  are correct, if not correct then the GATT_CMP_EVT message will be generically built
 *  and sent to Application directly. An error status is also possible be either 
 *  GATT_INVALID_PARAM_ERR or GATT_INVALID_TYPE_IN_SVC_SEARCH.
 *  If parameter is correct the GATT_DISC_CHAR_ALL_CMP_EVT or GATT_DISC_CHAR_BY_UUID_CMP_EVT
 *  message deceided by req_type will be received with the searched UUID, start handle and 
 *  end handle together.
 *
 ****************************************************************************************
 */
void app_gatt_disc_char_req(uint8_t req_type, uint16_t conhdl)
{
    struct gatt_disc_char_req *msg = KE_MSG_ALLOC(GATT_DISC_CHAR_REQ, TASK_GATT, TASK_APP,
                                                  gatt_disc_char_req);
    //Connection handle
    msg->conhdl = conhdl;
    //GATT request type
    msg->req_type = req_type;
    //Start handle range
    msg->start_hdl = 0x0001;
    //End handle range
    msg->end_hdl = GATT_MAX_ATTR_HDL;
    //Desired service: if 0x0000, discover all
    msg->desired_char.value_size = GATT_UUID_2_OCTET_LEN;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Characteristic descriptor discovery request. *//**
 *
 * @param[in] conhdl        Connection handle.
 * @response  GATT_DISC_CHAR_DESC_CMP_EVT and GATT_CMP_EVT
 * @description
 *
 *  This API is used by the application to send a GATT_DISC_CHAR_DESC_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If parameter is correct,
 *  the GATT_DISC_CHAR_DESC_CMP_EVT message will be received. 
 *
 ****************************************************************************************
 */
void app_gatt_disc_char_desc_req(uint16_t conhdl)
{
    struct gatt_disc_char_desc_req *msg = KE_MSG_ALLOC(GATT_DISC_CHAR_DESC_REQ, TASK_GATT, TASK_APP,
                                                       gatt_disc_char_desc_req);

    //Connection handle
    msg->conhdl = conhdl;
    //Start handle range
    msg->start_hdl = 0x0001;
    //End handle range
    msg->end_hdl = GATT_MAX_ATTR_HDL;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Read Characteristic request. *//**
 *
 * @param[in] req_type      GATT request type:
 * - GATT_READ_CHAR
 * - GATT_READ_BY_UUID_CHAR
 * - GATT_READ_LONG_CHAR
 * - GATT_READ_MULT_LONG_CHAR
 * - GATT_READ_DESC
 * - GATT_READ_LONG_DESC
 * @param[in] conhdl        Connection handle.
 * @param[in] valhdl        Value handle.
 * @response  GATT_READ_CHAR_RESP or GATT_READ_CHAR_MULTI_RESP
 * @description
 *
 *  This API is used by the application to send a GATT_READ_CHAR_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If parameter is correct,
 *  the GATT_READ_CHAR_RESP message will be received.
 *
 ****************************************************************************************
 */
void app_gatt_read_char_req(uint8_t req_type, uint16_t conhdl, uint16_t valhdl)
{
    struct gatt_read_char_req *msg = KE_MSG_ALLOC(GATT_READ_CHAR_REQ, TASK_GATT, TASK_APP,
                                                  gatt_read_char_req);

    //Connection handle
    msg->conhdl = conhdl;
    //GATT request type
    msg->req_type = req_type;
    //Read offset
    msg->offset = 0;
    //Start handle range
    msg->start_hdl = 0x0001;
    //End handle range
    msg->end_hdl = GATT_MAX_ATTR_HDL;
    //Number of UUID
    msg->nb_uuid = 0x01;
    //Handle of char value
    msg->uuid[0].expect_resp_size = ATT_UUID_16_LEN;
    msg->uuid[0].value_size = ATT_UUID_16_LEN;
    co_write16p(&msg->uuid[0].value[0], valhdl);

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Write Characteristic request. *//**
 *
 * @param[in] req_type      GATT request type:
 * - GATT_WRITE_NO_RESPONSE
 * - GATT_WRITE_SIGNED
 * - GATT_WRITE_CHAR
 * - GATT_WRITE_LONG_CHAR
 * - GATT_WRITE_DESC
 * - GATT_WRITE_LONG_DESC
 * @param[in] conhdl        Connection handle.
 * @param[in] valhdl        Value handle.
 * @param[in] val_len       Value length.
 * @param[in] pdata         Pointer to data.
 * @response  GATT_WRITE_CHAR_RESP
 * @description
 *
 *  This API is used by the application to send a GATT_WRITE_CHAR_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If parameter is correct,
 *  the GATT_WRITE_CHAR_RESP message will be received.
 *
 ****************************************************************************************
 */
void app_gatt_write_char_req(uint8_t req_type, uint16_t conhdl, uint16_t valhdl, uint16_t val_len, uint8_t *pdata)
{
    struct gatt_write_char_req *msg = KE_MSG_ALLOC_DYN(GATT_WRITE_CHAR_REQ, TASK_GATT, TASK_APP,
                                                   gatt_write_char_req, val_len);

    //Connection handle
    msg->conhdl = conhdl;
    //Offset to write
    msg->wr_offset = 0;
    //GATT request type
    msg->req_type = req_type;
    //Execute write
    msg->auto_execute = 0;
    //Valid characteristic handle
    msg->charhdl = valhdl;
    //Size of the value data
    msg->val_len = val_len;
    memcpy(&msg->value[0], pdata, val_len);

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Write reliable characteristic request. *//**
 *
 * @param[in] conhdl        Connection handle.
 * @param[in] nb_writes     Number of reliable writes.
 * @param[in] auto_execute  Automatic execute write or not(0x00 don't execute, 0x01 write).
 * @param[in] data_write    Pointer to the array of struct gatt_write_reliable_req.
 * @response  GATT_WRITE_CHAR_RELIABLE_RESP or GATT_CMP_EVT
 * @description
 *
 *  This API is used by the application to send a GATT_WRITE_RELIABLE_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If parameter is correct,
 *  the GATT_WRITE_CHAR_RELIABLE_RESP message will be received.
 *
 ****************************************************************************************
 */
void app_gatt_write_reliable_req(uint16_t conhdl, uint8_t nb_writes, uint8_t auto_execute, struct gatt_reliable_write *data_write)
{
    struct gatt_write_reliable_req *msg = KE_MSG_ALLOC_DYN(GATT_WRITE_RELIABLE_REQ, TASK_GATT, TASK_APP,
                                                       gatt_write_reliable_req, nb_writes * sizeof(struct gatt_reliable_write));

    //Connection handle
    msg->conhdl = conhdl;
    //Number of reliable writes
    msg->nb_writes = nb_writes;
    //Automatic execute write
    msg->auto_execute = auto_execute;
    memcpy(msg->reliable_write, data_write, nb_writes * sizeof(struct gatt_reliable_write));

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Write reliable characteristic request. *//**
 *
 * @param[in] conhdl        Connection handle.
 * @param[in] exe_wr_ena    Option flag to indicate for write or cancel(0x00 cancel, 0x01 write).
 * @response  GATT_CANCEL_WRITE_CHAR_RESP
 * @description
 *
 *  This API is used by the application to send a GATT_EXECUTE_WRITE_CHAR_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If parameter is correct,
 *  the GATT_CANCEL_WRITE_CHAR_RESP message will be received if cancel the reliable write.
 *
 ****************************************************************************************
 */
void app_gatt_execute_write_char_req(uint16_t conhdl, uint8_t exe_wr_ena)
{
    struct gatt_execute_write_char_req *msg = KE_MSG_ALLOC(GATT_EXECUTE_WRITE_CHAR_REQ, TASK_GATT, TASK_APP,
                                                           gatt_execute_write_char_req);

    //Connection handle
    msg->conhdl = conhdl;
    //Flag to describe if write or cancel
    msg->exe_wr_ena = exe_wr_ena;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Send notification request. *//**
 *
 * @param[in] conhdl        Connection handle.
 * @param[in] charhdl       Characteristic value handle.
 * @response  GATT_NOTIFY_CMP_EVT or none
 * @description
 *
 *  This API is used by the application to send a GATT_NOTIFY_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If permission is not allowed,
 *  the GATT_NOTIFY_CMP_EVT message will be received or application will not receive any mesage.
 *
 ****************************************************************************************
 */
void app_gatt_notify_req(uint16_t conhdl, uint16_t charhdl)
{
    struct gatt_notify_req *msg = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT, TASK_APP,
                                               gatt_notify_req);

    //Connection handle
    msg->conhdl = conhdl;
    //Characteristic handle
    msg->charhdl = charhdl;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Gatt Send indicate request. *//**
 *
 * @param[in] conhdl        Connection handle.
 * @param[in] charhdl       Characteristic value handle.
 * @response  GATT_HANDLE_VALUE_CFM
 * @description
 *
 *  This API is used by the application to send a GATT_INDICATE_REQ mssage.
 *  Upon reception of this message, GATT will checks whether the parameters are correct,
 *  if not correct then the GATT_CMP_EVT message with error code GATT_INVALID_PARAM_ERR 
 *  will be generically built and sent to Application directly. If parameter is correct,
 *  the GATT_HANDLE_VALUE_CFM message will be received.
 *
 ****************************************************************************************
 */
void app_gatt_indicate_req(uint16_t conhdl, uint16_t charhdl)
{
    struct gatt_indicate_req *msg = KE_MSG_ALLOC(GATT_INDICATE_REQ, TASK_GATT, TASK_APP,
                                                 gatt_indicate_req);

    //Connection handle
    msg->conhdl = conhdl;
    //Characteristic handle
    msg->charhdl = charhdl;

    ke_msg_send(msg);
}



#endif
/*
 ****************************************************************************************
 * @brief Gatt Send read request confirmation. *//**
 *
 * @param[in] conhdl        Connection handle.
 * @param[in] charhdl       Characteristic value handle.
 * @response  GATT_HANDLE_VALUE_CFM
 * @description
 *
 *  This API is used by the application to send a GATT_READ_REQ_CFM mssage.
 ****************************************************************************************
 */
void app_gatt_read_req_cfm(uint16_t conhdl, uint16_t atthdl, uint16_t val_len, uint8_t *value)
{
    struct gatt_read_req_cfm *cfm = KE_MSG_ALLOC_DYN(GATT_READ_REQ_CFM, TASK_GATT,
                                                   TASK_APP, gatt_read_req_cfm, val_len-1);
    cfm->conhdl = conhdl;
    cfm->atthdl = atthdl;
    cfm->val_len = val_len;
    memcpy(cfm->value, value, val_len);
    ke_msg_send(cfm);
}
/// @} APP_GATT_API

