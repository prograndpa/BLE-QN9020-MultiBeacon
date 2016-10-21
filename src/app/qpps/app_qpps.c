/**
 ****************************************************************************************
 *
 * @file app_qpps.c
 *
 * @brief Application QPPS API
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
 * @addtogroup APP_QPPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_QPP_SERVER
#include "app_qpps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create Quintic Private Profile service database - at initiation   *//**
 *
 * @param[in] char_num The number of Characteristic used to send data
 *
 * @response QPPS_CREATE_DB_CFM
 * @description
 * This function shall be used to add an instance of the Quintic Private Profile service into 
 * the database. This should be done during the initialization phase of the device.
 * @note
 * Application can define the number of Characteristic used to send data to client through
 * notify.
 * 
 ****************************************************************************************
 */
void app_qpps_create_db(uint8_t char_num)
{
    struct qpps_create_db_req * msg = KE_MSG_ALLOC(QPPS_CREATE_DB_REQ, TASK_QPPS, TASK_APP, qpps_create_db_req);

    msg->tx_char_num = char_num;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the Quintic Private profile - at connection      *//**
 * 
 * @param[in] conhdl Connection handle.
 * @param[in] sec_lvl Security level required for protection of HRS attributes:
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * @param[in] ntf_en Notification configuration
 *
 * @response None
 * @description
 * This function is used for enabling the Server role of the Quintic Private service.
 * 
 ****************************************************************************************
 */
void app_qpps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t ntf_en)
{
    struct qpps_enable_req * msg = KE_MSG_ALLOC(QPPS_ENABLE_REQ, TASK_QPPS, TASK_APP,
                                                qpps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->ntf_en = ntf_en;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send a notification containing raw data - at connection.        *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] index Index of Characteristic to be sent
 * @param[in] length Length of data to be sent
 * @param[in] data Pointer to data to be sent
 *
 * @response QPPS_DATA_SEND_CFM
 * @description
 * This function is used by the application to send a raw data.
 * 
 ****************************************************************************************
 */
void app_qpps_data_send(uint16_t conhdl, uint8_t index, uint8_t length, uint8_t *data)
{
    struct qpps_data_send_req * msg = KE_MSG_ALLOC_DYN(QPPS_DATA_SEND_REQ, TASK_QPPS, TASK_APP,
                                                       qpps_data_send_req, length);

    msg->conhdl = conhdl;
    msg->index = index;
    msg->length = length;
    memcpy(msg->data, data, length);

    ke_msg_send(msg);
}

#endif // BLE_QPP_SERVER

/// @} APP_QPPS_API
