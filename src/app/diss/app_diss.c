/**
 ****************************************************************************************
 *
 * @file app_diss.c
 *
 * @brief Application DISS API
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
 * @addtogroup APP_DISS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_DIS_SERVER)
#include "app_diss.h"


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
 ****************************************************************************************
 * @brief Create device information service database    *//**
 * 
 * @param[in] features Indicate characteristics that are supported, possible values are
 * - DIS_MANUFACTURER_NAME_CHAR_SUP
 * - DIS_MODEL_NB_STR_CHAR_SUP
 * - DIS_SERIAL_NB_STR_CHAR_SUP
 * - DIS_HARD_REV_STR_CHAR_SUP
 * - DIS_FIRM_REV_STR_CHAR_SUP
 * - DIS_SW_REV_STR_CHAR_SUP
 * - DIS_SYSTEM_ID_CHAR_SUP
 * - DIS_IEEE_CHAR_SUP
 * - DIS_PNP_ID_CHAR_SUP
 *
 * @response DISS_CREATE_DB_CFM
 * @description
 * This function shall be used to add an instance of the Device Information Service into 
 * the database. This should be done during the initialization phase of the device.
 * @note All characteristics of the Device Information Service are optional. However, some profiles
 * require the presence of several of these characteristics. Please refer to the specification of
 * these profiles for more information.
 ****************************************************************************************
 */
void app_diss_create_db(uint16_t features)
{
    struct diss_create_db_req *msg = KE_MSG_ALLOC(DISS_CREATE_DB_REQ, TASK_DISS, TASK_APP, diss_create_db_req);

    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Enable device information service     *//**
 *
 * @param[in] conhdl Connection handle for which the profile Target role is enabled
 * @param[in] sec_lvl Security level required for protection of attributes,
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 *
 * @response None
 * @description
 * This function is used for enabling the Server role of the device information service
 ****************************************************************************************
 */
void app_diss_enable_req(uint16_t conhdl, uint8_t sec_lvl)
{
    struct diss_enable_req * msg = KE_MSG_ALLOC(DISS_ENABLE_REQ, TASK_DISS, TASK_APP,
                                                 diss_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Set Characteristic value     *//**
 *
 * @param[in] char_code Characteristic Code, possible values are:
 * - DIS_MANUFACTURER_NAME_CHAR
 * - DIS_MODEL_NB_STR_CHAR
 * - DIS_SERIAL_NB_STR_CHAR
 * - DIS_HARD_REV_STR_CHAR
 * - DIS_FIRM_REV_STR_CHAR
 * - DIS_SW_REV_STR_CHAR
 * - DIS_SYSTEM_ID_CHAR
 * - DIS_IEEE_CHAR
 * - DIS_PNP_ID_CHAR
 *
 * @param[in] val_len Value length
 * @param[in] val Pointer to value
 *
 * @response None
 * @description
 * This function is used to initialize any of the characteristic values before a connection with a peer
 * device.
 ****************************************************************************************
 */
void app_set_char_val_req(uint8_t char_code, uint8_t val_len, uint8_t const *val)
{
    struct diss_set_char_val_req * msg = KE_MSG_ALLOC_DYN(DISS_SET_CHAR_VAL_REQ, TASK_DISS, TASK_APP,
                                                          diss_set_char_val_req, val_len);

    msg->char_code = char_code;
    msg->val_len = val_len;
    memcpy(msg->val, val, val_len);
    ke_msg_send(msg);
}

#endif // BLE_DIS_SERVER

/// @} APP_DISS_API
