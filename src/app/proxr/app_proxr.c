/**
 ****************************************************************************************
 *
 * @file app_proxr.c
 *
 * @brief Application PROXR API
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
 * @addtogroup APP_PROXR_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_PROX_REPORTER
#include "app_proxr.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the proximity reporter service database - at initiation    *//**
 *
 * @param[in] features Indicate if optional features is supported or not, possible values are:
 * PROXR_IAS_TXPS_NOT_SUP
 * PROXR_IAS_TXPS_SUP
 *
 * @response PROXR_CREATE_DB_CFM
 * @description
 * This function shall be used to after reception of create database. This should be done
 * during the initialization phase of the device. The status parameter indicates if the
 * services have been successfully added (ATT_ERR_NO_ERROR) or not (ATT_INSUFF_RESOURCE).
 ****************************************************************************************
 */
void app_proxr_create_db(uint8_t features)
{
    struct proxr_create_db_req * msg = KE_MSG_ALLOC(PROXR_CREATE_DB_REQ, TASK_PROXR, TASK_APP,
                                                    proxr_create_db_req);

    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the proximity reporter role - at connection     *//**
 *
 * @param[in] conhdl Connection handle for which the profile Reporter role is enabled
 * @param[in] sec_lvl Security level required for protection of attributes.
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] lls_alert_lvl Saved value for LLS alert level, from previous profile use.
 * 0 if the connection is a configuration connection.
 * @param[in] txp_lvl TX Power level, range from -100 to 20 
 *
 * @response None
 * @description
 * This function is used for enabling the Reporter role of the Proximity profile.
 * After calling this function, the services are unhidden from the peer discove.
 ****************************************************************************************
 */
void app_proxr_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t lls_alert_lvl, int8_t txp_lvl)
{
    struct proxr_enable_req * msg = KE_MSG_ALLOC(PROXR_ENABLE_REQ, TASK_PROXR, TASK_APP,
                                                 proxr_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->lls_alert_lvl = lls_alert_lvl;
    msg->txp_lvl = txp_lvl;
    ke_msg_send(msg);
}

#endif // BLE_PROX_REPORTER

/// @} APP_PROXR_API
