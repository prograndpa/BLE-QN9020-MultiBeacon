/**
 ****************************************************************************************
 *
 * @file app_hogprh.c
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
 * @addtogroup APP_HOGPRH_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HID_REPORT_HOST
#include "app_hogprh.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the HID Over GATT profile - at connection. *//**
 *
 * @param[in] hids_nb   Number of instances of the HID Service that have been found during the last discovery. 
 * @param[in] hids      Information about the HID Service instances that have been found during the last discovery. 
 * @param[in] conhdl    Connection handle 
 * @response HOGPRH_ENABLE_CFM 
 * @description
 *
 * This API is used for enabling the Report Host role of the HOGP. This function 
 * contains BLE connection handle, the connection type and the previously saved discovered HIDS details on peer. 
 *
 * The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration or PRF_CON_NORMAL 
 * (0x01) for a normal connection with a bonded device.  Application shall save those information to reuse them for 
 * other connections. During normal connection, previously discovered device information can be reused. 
 *
 * If it is a discovery /configuration type of connection, it is useless to fill the HIDS parameters (hids_nb and hids) are 
 * useless. Otherwise they will contain pertinent data which will be kept in the Boot Host environment while enabled. 
 *  
 * For a normal connection, the response to this request is sent right away after saving the HIDS content in the 
 * environment and registering HOGPRH in GATT to receive the notifications for the known attribute handles in HIDS that 
 * would be notified (Report Characteristic). For a discovery connection, discovery of the peer HIDS is started and the 
 * response will be sent at the end of the discovery with the discovered attribute details. 
 *
 ****************************************************************************************
 */
void app_hogprh_enable_req(uint8_t hids_nb, struct hogprh_hids_content *hids, uint16_t conhdl)
{
    struct hogprh_enable_req * msg = KE_MSG_ALLOC(HOGPRH_ENABLE_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                 hogprh_enable_req);

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
        memcpy(&msg->hids[0], hids, hids_nb*sizeof(struct hogprh_hids_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Disable profile role - at disconnection. *//**
 *
 * @param[in] conhdl    Connection handle 
 * @response None 
 * @description
 *
 *  This API is used for disabling the Report Host role of the HOGP. The Application sends it, and it 
 *  contains the connection handle for the connection this profile is activated. 
 *
 ****************************************************************************************
 */
void app_hogprh_disable_req(uint16_t conhdl)
{
    struct hogprh_disable_req *msg = KE_MSG_ALLOC(HOGPRH_DISABLE_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                hogprh_disable_req);
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a HIDS characteristic/descriptor value. *//**
 *
 * @param[in] read_code     Characteristic Code:
 *  - HOGPRH_RD_HIDS_REPORT_MAP
 *  - HOGPRH_RD_HIDS_HID_INFO
 *  - HOGPRH_RD_HIDS_PROTOCOL_MODE
 *  - HOGPRH_RD_HIDS_REPORT
 *  - HOGPRH_RD_HIDS_REPORT_MAP_EXT_REP_REF
 *  - HOGPRH_RD_HIDS_REPORT_REF 
 *  - HOGPRH_RD_HIDS_REPORT_CFG
 * @param[in] report_nb     Report Characteristic instance if needed
 * @param[in] hids_nb       HID Service instance 
 * @param[in] conhdl        Connection handle 
 * @response  
 *
 * The response depends on the read_code parameter value. If an error has been raised before this message 
 * is sent in the air, a HOGPRH_ERR_IND message is sent; if the received value doesn't match with requirements or 
 * implementation limitations a HOGPRH_RD_CHAR_ERR_RSP is sent. 
 *
 * @description
 *
 * This API shall be used to read the value of a characteristic or a descriptor in the HID Device 
 * database. 
 *
 * When the HOGP Report Host task receives this message, the handler checks several parameters. If one of these don't 
 * match requirements, a HOGPRH_ERROR_IND is sent to the application. The table below resumes the possible status 
 * values: 
 *
 * - PRF_ERR_INVALID_PARAM (0x81): Either the provided Connection Handle is unknown, or the specified 
 * HIDS instance is upper than the number of found HIDS, or the specified Report Characteristic instance is upper 
 * than the limitation. 
 * - PRF_ERR_INEXISTENT_HDL (0x82): The required attribute has not been found in the peer device database. 
 *
 ***************************************************************************************
 */
void app_hogprh_rd_char_req(uint8_t read_code, uint8_t report_nb, uint8_t hids_nb, uint16_t conhdl)
{
    struct hogprh_rd_char_req *msg = KE_MSG_ALLOC(HOGPRH_RD_CHAR_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                hogprh_rd_char_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;
    ///Read code
    msg->read_code = read_code;
    ///Report Char. Instance
    msg->report_nb = report_nb;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the characteristics. *//**
 *
 * @param[in] report_nb     Report Characteristic instance. 
 * @param[in] ntf_cfg       Configuration value to write: 
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] hids_nb       HID Service instance 
 * @param[in] conhdl        Connection handle 
 * @response   HOGPRH_WR_CHAR_RSP or HOGPRH_ERROR_IND
 * @description
 *
 *  This API shall be used to enable or disable the notifications for a Report Characteristic instance. 
 *  
 *  When the HOGP Report Host task receives this message, the handler checks several parameters. If one of these don't 
 *  match requirements, a HOGPRH_ERROR_IND is sent to the application. The table below resumes the possible status 
 *  values: 
 *  - PRF_ERR_INVALID_PARAM (0x81): Either the provided Connection Handle is unknown, or the specified 
 *  HIDS instance is upper than the number of found HIDS, or the specified 
 *  Report Characteristic instance is upper than the limitation or the ntf_cfg 
 *  parameter value is not valid. 
 *  - PRF_ERR_INEXISTENT_HDL (0x82): The required attribute has not been found in the peer device database. 
 *  Either the Report Characteristic instance doesn't exist in the peer device 
 *  database or this characteristic doesn't support notification (not an Input Report). 
 *
 ****************************************************************************************
 */
void app_hogprh_cfg_ntf_req(uint8_t report_nb, uint16_t ntf_cfg, uint8_t hids_nb, uint16_t conhdl)
{
    struct hogprh_cfg_ntf_req *msg = KE_MSG_ALLOC(HOGPRH_CFG_NTF_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                hogprh_cfg_ntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS instance
    msg->hids_nb = hids_nb;
    ///Report instance (The Report Char. is the only one for which we can receive notifications)
    msg->report_nb = report_nb;
    ///Stop/notify value to configure into the peer characteristic
    msg->ntf_cfg = ntf_cfg;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Write peer HID Control Point value request. *//**
 *
 * @param[in] ctnl_pt       HID Control Point value: 
 *  - HOGP_CTNL_PT_SUSPEND
 *  - HOGP_CTNL_PT_EXIT_SUSPEND
 * @param[in] hids_nb       HID Service instance 
 * @param[in] conhdl        Connection handle 
 * @response   None or HOGPRH_ERROR_IND 
 * @description
 *
 *  This API shall be used to write the HID Control Point Characteristic value in the peer device 
 *  database. 
 *
 *  When the HOGP Report Host task receives this message, the handler checks several parameters. If one of these don't 
 *  match requirements, a HOGPRH_ERROR_IND is sent to the application. The table below resumes the possible status 
 *  values: 
 *  - PRF_ERR_INVALID_PARAM (0x81): Either the provided Connection Handle is unknown, or the specified 
 *  HIDS instance is upper than the number of found HIDS or the 
 *  hid_ctnl_pt parameter value is not valid. 
 *  - PRF_ERR_INEXISTENT_HDL (0x82): The required attribute has not been found in the peer device database. 
 *  The HID Control Point Characteristic is mandatory. Thus, this error 
 *  should never happened if the discovery process has been successful. 
 *
 ****************************************************************************************
 */
void app_hogprh_hid_ctnl_pt_wr_req(uint8_t ctnl_pt, uint8_t hids_nb, uint16_t conhdl)
{
    struct hogprh_hid_ctnl_pt_wr_req *msg = KE_MSG_ALLOC(HOGPRH_HID_CTNL_PT_WR_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                hogprh_hid_ctnl_pt_wr_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;
    ///HID Control Point value to write
    msg->hid_ctnl_pt = ctnl_pt;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Write peer Protocol Mode value request (Report Protocol Mode). *//**
 *
 * @param[in] hids_nb       HID Service instance 
 * @param[in] conhdl        Connection handle 
 * @response   None or HOGPRH_ERROR_IND 
 * @description
 *
 *  This API shall be used to set the protocol mode of a HID Service instance to the Report Procotol 
 *  Mode. 
 *
 *  When the HOGP Report Host task receives this message, the handler checks several parameters. If one of these don't 
 *  match requirements, a HOGPRH_ERROR_IND is sent to the application. The table below resumes the possible status 
 *  values: 
 *  - PRF_ERR_INVALID_PARAM (0x81): Either the provided Connection Handle is unknown, or the specified 
 *  HIDS instance is upper than the number of found HIDS. 
 *  PRF_ERR_INEXISTENT_HDL (0x82): The required attribute has not been found in the peer device database. 
 *
 ****************************************************************************************
 */
void app_hogprh_set_report_proto_mode_req(uint8_t hids_nb, uint16_t conhdl)
{
    struct hogprh_set_report_proto_mode_req *msg = KE_MSG_ALLOC(HOGPRH_SET_REPORT_PROTO_MODE_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                hogprh_set_report_proto_mode_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Write peer Report value request. *//**
 *
 * @param[in] report_nb         Report Characteristic instance
 * @param[in] report_length     Length of the Report value to write
 * @param[in] out_report_type   Type of write to perform if the Report is an Output Report
 * @param[in] report            Report Characteristic value
 * @param[in] hids_nb           HID Service instance 
 * @param[in] conhdl            Connection handle 
 * @response   None or HOGPRH_WR_CHAR_RSP or HOGPRH_ERROR_IND 
 * @description
 *
 *  This API shall be used to write the value of a Report Characteristic in the peer device database.  
 *
 ****************************************************************************************
 */
void app_hogprh_report_wr_req(uint8_t report_nb, uint8_t report_length, 
    uint8_t out_report_type, uint8_t *report, uint8_t hids_nb, uint16_t conhdl)
{
    struct hogprh_report_wr_req *msg = KE_MSG_ALLOC_DYN(HOGPRH_REPORT_WR_REQ, KE_BUILD_ID(TASK_HOGPRH, conhdl), TASK_APP,
                                                hogprh_report_wr_req, report_length);
    ///Connection handle
    msg->conhdl = conhdl;
    ///HIDS Instance
    msg->hids_nb = hids_nb;
    ///Report instance
    msg->report_nb = report_nb;
    ///Output Report Write Type, May be GATT_WRITE_CHAR or GATT_WRITE_NO_RESPONSE
    msg->out_report_type = out_report_type;
    ///Report value
    memcpy(&msg->report[0], report, report_length);

    // Send the message
    ke_msg_send(msg);
}

#endif

/// @} APP_HOGPRH_API
