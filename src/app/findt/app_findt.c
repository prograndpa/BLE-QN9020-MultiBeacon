/**
 ****************************************************************************************
 *
 * @file app_findt.c
 *
 * @brief Application FINDT API
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
 * @addtogroup APP_FINDT_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_FINDME_TARGET
#include "app_findt.h"


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
 * @brief Create find me target database        *//**
 *
 * @response FINDT_CREATE_DB_CFM
 * @description
 * This functiuon shall be used to add an instance of the Immediate Alert Service into the
 * database. This should be done during the initialization phase of the device.
 * The status parameter indicates if the IAS has been successfully added or not.
 * Possible values for the status are: ATT_ERR_NO_ERROR and ATT_INSUFF_RESOURCE.
 ****************************************************************************************
 */
void app_findt_create_db(void)
{
    struct findt_create_db_req * msg = KE_MSG_ALLOC(FINDT_CREATE_DB_REQ, TASK_FINDT, TASK_APP,
                                                    findt_create_db_req);

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Enable find me target service     *//**
 *
 * @param[in] conhdl Connection handle for which the profile Target role is enabled
 * @param[in] sec_lvl Security level required for protection of attributes 
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @response None
 * @description
 * This functiuon is used for enabling the Target role of the Find Me profile.
 * It contains the connection handle for the connection this profile is activated.
 ****************************************************************************************
 */
void app_findt_enable_req(uint16_t conhdl, uint8_t sec_lvl)
{
    struct findt_enable_req * msg = KE_MSG_ALLOC(FINDT_ENABLE_REQ, TASK_FINDT, TASK_APP,
                                                 findt_enable_req);
    
    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    ke_msg_send(msg);
}

#endif // BLE_FINDME_TARGET

/// @} APP_FINDT_API
