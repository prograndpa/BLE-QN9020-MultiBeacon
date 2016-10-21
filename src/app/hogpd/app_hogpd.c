/**
 ****************************************************************************************
 *
 * @file app_hogpd.c
 *
 * @brief Application HOGPD API
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
 * @addtogroup APP_HOGPD_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HID_DEVICE
#include "app_hogpd.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the hid device database - at initiation        *//**
 *
 * @param[in] hids_nb Number of HID Service instances to add in the database
 * @param[in] cfg Pointer to the struct hogpd_hids_cfg containing Configuration for each
 * HID Service you want to add
 *
 * @response HOGPD_CREATE_DB_CFM
 * @description
 * This function shall be used to add one or more instance of the HID Service in the database.
 * @note Multiples service instances of the HID Service should allow implementers to define
 * HID Devices whose combined functions require more than 512 octets of data to describe.
 * Thus, the second instance of the HID Service shall exist only if the Report Characteristic
 * value exceeds 512 bytes.
 ****************************************************************************************
 */
void app_hogpd_create_db(uint8_t hids_nb, struct hogpd_hids_cfg *cfg)
{
    struct hogpd_create_db_req * msg = KE_MSG_ALLOC(HOGPD_CREATE_DB_REQ, TASK_HOGPD, TASK_APP, hogpd_create_db_req);

    msg->hids_nb = hids_nb;
    while (hids_nb--)
        msg->cfg[hids_nb] = cfg[hids_nb];
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Set the HID Report Descriptor as Report Map Characteristic value      *//**
 *
 * @param[in] report_map_len Length of the Report Map Characteristic value
 * @param[in] hids_nb HID Service instance the Report Map Characteristic belongs to
 * @param[in] report_map Pointer to the Report Map Characteristic value
 *
 * @response None or HOGPD_ERROR_IND
 * @description
 * This function shall be used to initialize the Report Map Characteristic value in the
 * database. This value is not supposed to change during the connection or during the
 * device life cycle.
 * According to the BLE HOGP specification, the Report Characteristic value length is
 * limited to 512 bytes. If the value has been set with success in the database, no
 * response is sent to the application. If an error is raised, a HOGPD_ERROR_IND
 * message will be sent with one the following error status:
 * PRF_ERR_INVALID_PARAM The specified Report Map Characteristic value length is upper
 * than the limitation(512 bytes by default)
 * PRF_ERR_REQ_DISALLOWED The required HID Service has not been added in the database
 ****************************************************************************************
 */
void app_hogpd_report_map_req(uint16_t report_map_len, uint8_t hids_nb, uint8_t *report_map)
{
    struct hogpd_set_report_map_req * msg = KE_MSG_ALLOC_DYN(HOGPD_SET_REPORT_MAP_REQ, TASK_HOGPD, TASK_APP, 
                                            hogpd_set_report_map_req, report_map_len);

    msg->report_map_len = report_map_len;   // Report Map Length
    msg->hids_nb = hids_nb;                 // HIDS Instance
    memcpy(msg->report_map, report_map, report_map_len);
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the HID profile - at connection     *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] sec_lvl Required security level. Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type
 * @param[in] hids_nb Number of HID Service instances
 * @param[in] ntf_cfg Saved notification configurations
 *
 * @response None or HOGPD_ERROR_IND
 * @description
 * This function shall be used after the connection with a peer device has been established
 * in order to enable the HOGP device role task for the specified connection.
 ****************************************************************************************
 */
void app_hogpd_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint8_t hids_nb, struct hogpd_hids_ntf_cfg *ntf_cfg)
{
    struct hogpd_enable_req * msg = KE_MSG_ALLOC(HOGPD_ENABLE_REQ, TASK_HOGPD, TASK_APP, hogpd_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    while (hids_nb--)
        msg->ntf_cfg[hids_nb] = ntf_cfg[hids_nb];
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Request sending of a report to the host       *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] hids_nb HID Service instance
 * @param[in] report_nb Report Characteristic instance
 * @param[in] report_length Length of the Report Characteristic value
 * @param[in] report Report Characteristic value
 *
 * @response HOGPD_NTF_SENT_CFM
 * @description
 * This function is used to update the value of the Report Characteristic stored in the
 * database and to notify the Host about this new value if sending of notifications has
 * been enabled for it.
 ****************************************************************************************
 */
void app_hogpd_report_upd_req(uint16_t conhdl, uint8_t hids_nb, uint8_t report_nb, uint8_t report_length, uint8_t *report)
{
    struct hogpd_report_info * msg = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ, TASK_HOGPD, TASK_APP, 
                                                      hogpd_report_info, report_length);

    msg->conhdl = conhdl;
    msg->hids_nb = hids_nb;
    msg->report_nb = report_nb; // Report Char. Instance
    msg->report_length = report_length;
    memcpy(msg->report, report, report_length);
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Request sending of a boot report to the host       *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] hids_nb HID Service instance
 * @param[in] char_code Characteristic code is used to indicate keyboard or mouse data, possible values are:
 * - HOGPD_BOOT_KB_IN_REPORT_CHAR
 * - HOGPD_BOOT_MOUSE_IN_REPORT_CHAR
 * - HOGPD_BOOT_KB_OUT_REPORT_CHAR
 * @param[in] report_length Report data length
 * @param[in] boot_report Boot Report Characteristic value
 *
 * @response HOGPD_NTF_SENT_CFM
 * @description
 * This function is used to update the value of the Boot Report Characteristic value
 * stored in the database and to notify the Host about this new value if sending of 
 * notifications has been enabled for it.
 ****************************************************************************************
 */
void app_hogpd_boot_report_upd_req(uint16_t conhdl, uint8_t hids_nb, uint8_t char_code, uint8_t report_length, uint8_t *boot_report)
{
    struct hogpd_boot_report_info * msg = KE_MSG_ALLOC_DYN(HOGPD_BOOT_REPORT_UPD_REQ, TASK_HOGPD, TASK_APP, 
                                                       hogpd_boot_report_info, report_length);

    msg->conhdl = conhdl;
    msg->hids_nb = hids_nb;
    msg->char_code = char_code;
    msg->report_length = report_length;
    memcpy(msg->boot_report, boot_report, report_length);                       
    ke_msg_send(msg);
}

#endif // BLE_HID_DEVICE

/// @} APP_HOGPD_API

