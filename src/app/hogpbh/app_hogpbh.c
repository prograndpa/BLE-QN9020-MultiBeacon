/**
 ****************************************************************************************
 *
 * @file app_hogpbh.c
 *
 * @brief Application HID Over GATT Profile Boot Host API
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
 * @addtogroup APP_HOGPBH_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HID_BOOT_HOST
#include "app_hogpbh.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/*
 ****************************************************************************************
 * @brief Start the HID Over GATT profile - at connection. *//**
 *
 * @param[in] hids_nb  Number of instances of the HID Service that have been found during the last discovery
 * @param[in] hids     Information about HID Services that have been found during the last discovery
 * @param[in] conhdl   Connection handle
 * @response  HOGPBH_ENABLE_CFM 
 * @description
 *
 *  This API is used for enabling the Boot Host role of the HOGP. This function contains 
 *  BLE connection handle, the connection type and the previously saved discovered HIDS details on peer. 
 *
 *  The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration or PRF_CON_NORMAL 
 *  (0x01) for a normal connection with a bonded device. Application shall save those information to reuse them for 
 *  other connections. During normal connection, previously discovered device information can be reused. 
 *
 *  If it is a discovery/configuration type of connection, it is useless to fill the HIDS parameters (hids_nb and hids) are 
 *  useless. Otherwise they will contain pertinent data which will be kept in the Boot Host environment while enabled. 
 *
 *  For a normal connection, the response to this request is sent right away after saving the HIDS content in the 
 *  environment and registering HOGPBH in GATT to receive the notifications for the known attribute handles in HIDS that 
 *  would be notified (Boot Keyboard Input Report and Boot Mouse Input Report). For a discovery connection, discovery 
 *  of the peer HIDS is started and the response will be sent at the end of the discovery with the discovered attribute 
 *  details. 
 *
 ****************************************************************************************
 */
void app_hogpbh_enable_req(uint8_t hids_nb, struct hids_content *hids, uint16_t conhdl)
{
    struct hogpbh_enable_req * msg = KE_MSG_ALLOC(HOGPBH_ENABLE_REQ, KE_BUILD_ID(TASK_HOGPBH, conhdl), TASK_APP,
                                                 hogpbh_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Number of HIDS instances
    msg->hids_nb = hids_nb;
    ///Connection type
    if (hids == NULL || hids_nb == 0)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->hids[0], hids, hids_nb*sizeof(struct hids_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Disable profile role - at disconnection. *//**
 *
 * @param[in] conhdl   Connection handle
 * @response  None 
 * @description
 *
 *  This API  is used for disabling the Boot Host role of the HOGP. The function 
 *  contains the connection handle for the connection this profile is activated. 
 *
 ****************************************************************************************
 */
void app_hogpbh_disable_req(uint16_t conhdl)
{
    struct hogpbh_disable_req *msg = KE_MSG_ALLOC(HOGPBH_DISABLE_REQ, KE_BUILD_ID(TASK_HOGPBH, conhdl), TASK_APP,
                                                hogpbh_disable_req);
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a HIDS or DIS or BAS characteristic value. *//**
 *
 * @param[in] char_code     Characteristic value code: 
    - HOGPBH_RD_HIDS_PROTO_MODE
    - HOGPBH_RD_HIDS_BOOT_KB_IN_REPORT
    - HOGPBH_RD_HIDS_BOOT_KB_OUT_REPORT
    - HOGPBH_RD_HIDS_BOOT_MOUSE_IN_REPORT
    - HOGPBH_RD_HIDS_BOOT_KB_IN_REPORT_CFG
    - HOGPBH_RD_HIDS_BOOT_MOUSE_IN_REPORT_CFG
 * @param[in] hids_nb       HIDS instance
 * @param[in] conhdl        Connection handle
 * @response  
 *
 *  The response depends on the read_code parameter value. If an error has been raised before this message 
 *  is sent in the air, a HOGPBH_ERR_IND message is sent; if the received value doesn't match with requirements or 
 *  implementation limitations a HOGPBH_RD_CHAR_ERR_RSP is sent. 
 * @description
 *
 *  This API shall be used to read the value of a characteristic or a descriptor in the HID Device 
 *  database. 
 *
 *  When the HOGP Boot Host task receives this message, the handler checks several parameters. If one of these don't 
 *  match requirements, a HOGPBH_ERROR_IND is sent to the application. The table below resumes the possible status 
 *  values: 
 *  - PRF_ERR_INVALID_PARAM (0x81): Either the provided Connection Handle is unknown, or the specified 
 *  HIDS instance is upper than the number of found HIDS. 
 *  - PRF_ERR_INEXISTENT_HDL (0x82): The required attribute has not been found in the peer device database. 
 * 
 ****************************************************************************************
 */
void app_hogpbh_rd_char_req(uint8_t char_code, uint8_t hids_nb, uint16_t conhdl)
{
    struct hogpbh_rd_char_req *msg = KE_MSG_ALLOC(HOGPBH_RD_CHAR_REQ, KE_BUILD_ID(TASK_HOGPBH, conhdl), TASK_APP,
                                                hogpbh_rd_char_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;
    ///Characteristic value code
    msg->char_code = char_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the characteristics. *//**
 *
 * @param[in] desc_code     Client Characteristic Configuration Descriptor Code :
 *  - HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG,
 *  - HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG,
 * @param[in] ntf_cfg       Configuration value to write: 
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] hids_nb       HIDS instance
 * @param[in] conhdl        Connection handle
 * @response  HOGPBH_WR_CHAR_RSP or HOGPBH_ERROR_IND
 * @description 
 *
 *  This API shall be used to enable or disable the notifications for either the Boot Keyboard Input 
 *  Characteristic or the Boot Mouse Input Characteristic. 
 *
 *  When the HOGP Boot Host task receives this message, the handler checks several parameters. If one of these don't 
 *  match requirements, a HOGPBH_ERROR_IND is sent to the application. The table below resumes the possible status 
 *  values: 
 *  - PRF_ERR_INVALID_PARAM (0x81): Either the provided Connection Handle is unknown, or the specified 
 *  HIDS instance is upper than the number of found HIDS, or the ntf_cfg parameter value is not valid. 
 *  - PRF_ERR_INEXISTENT_HDL (0x82): The required attribute has not been found in the peer device database. 
 *
 ****************************************************************************************
 */
void app_hogpbh_cfg_ntf_req(uint8_t desc_code, uint16_t ntf_cfg, uint8_t hids_nb, uint16_t conhdl)
{
    struct hogpbh_cfg_ntf_req *msg = KE_MSG_ALLOC(HOGPBH_CFG_NTF_REQ, KE_BUILD_ID(TASK_HOGPBH, conhdl), TASK_APP,
                                                hogpbh_cfg_ntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    /// HIDS instance
    msg->hids_nb = hids_nb;
    ///Client Characteristic Configuration Code
    msg->desc_code = desc_code;
    ///Stop/notify value to configure into the peer characteristic
    msg->ntf_cfg = ntf_cfg;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Write peer Boot Keyboard Output Report Char. value. *//**
 * @param[in] wr_type       Write type (Write or Write without Response)
 * @param[in] char_code     char code
 * @param[in] report_length Report data length
 * @param[in] hids_nb       HIDS instance
 * @param[in] report        Boot Keyboard Output Report Characteristic value to write 
 * @param[in] conhdl        Connection handle
 * @response  None or HOGPBH_WR_CHAR_RSP
 * @description  
 *
 *  This API shall be used to write the value of a Boot Keyboard Output Report Characteristic in the 
 *  peer device database. 
 *
 ****************************************************************************************
 */
void app_hogpbh_boot_report_wr_req(uint8_t wr_type, uint8_t char_code, uint8_t report_length, uint8_t hids_nb, uint8_t *report, uint16_t conhdl)
{
    struct hogpbh_boot_report_wr_req *msg = KE_MSG_ALLOC(HOGPBH_BOOT_REPORT_WR_REQ, KE_BUILD_ID(TASK_HOGPBH, conhdl), TASK_APP,
                                                         hogpbh_boot_report_wr_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;
    msg->char_code = char_code;
    ///Write type (Write or Write without Response)
    ///GATT_WRITE_CHAR  or GATT_WRITE_NO_RESPONSE
    msg->wr_type = wr_type;
    msg->report_length = report_length;
    ///Boot Keyboard Output Report
    memcpy(&msg->report[0], report, report_length);

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Set Boot Protocol Mode on peer device. *//**
 *
 * @param[in] hids_nb       HIDS instance
 * @param[in] conhdl        Connection handle
 * @response  HOGPBH_WR_CHAR_RSP or HOGPBH_ERROR_IND
 * @description  
 *
 *  This API shall be used to set  the protocol mode of a  HID Service instance to the Boot  Procotol 
 *  Mode. 
 *
 *  The  default  protocol  mode  for  a  HID Device  able  to  support  either  the  Boot  protocol  mode  or  the  Report  protocol 
 *  mode  (the  Protocol  Mode  characteristic  shall  be  present  in  its  database)  is  the  Report  protocol  mode.  Thus,  the 
 *  application shall send this message right after the end of the discovery. 
 *
 *  When the HOGP Boot Host task receives this message, the handler checks several parameters. If one of these don't 
 *  match requirements, a  HOGPBH_ERROR_IND is sent  to the application. The table below resumes the possible status 
 *  values: 
 *  - PRF_ERR_INVALID_PARAM  0x81  Either  the  provided  Connection  Handle  is  unknown,  or  the  specified 
 *  HIDS instance is upper than the number of found HIDS. 
 *  - PRF_ERR_INEXISTENT_HDL  0x82  The required attribute has not been found in the peer device database. 
 *
 ****************************************************************************************
 */
void app_hogpbh_set_boot_proto_mode_req(uint8_t hids_nb, uint16_t conhdl)
{
    struct hogpbh_set_boot_proto_mode_req *msg = KE_MSG_ALLOC(HOGPBH_SET_BOOT_PROTO_MODE_REQ, KE_BUILD_ID(TASK_HOGPBH, conhdl), TASK_APP,
                                                hogpbh_set_boot_proto_mode_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;

    // Send the message
    ke_msg_send(msg);
}
#endif 

/// @} APP_HOGPBH_API
