/**
 ****************************************************************************************
 *
 * @file app_findt.h
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

#ifndef APP_FINDT_H_
#define APP_FINDT_H_

/**
 ****************************************************************************************
 * @addtogroup APP_FINDT_API Find Me Target
 * @ingroup APP_FINDT
 * @brief Find Me Profile Target.
 *
 * The Find Me profile defines the behavior when a button is pressed on a device to cause
 * an immediate alert on a peer device. This can be used to allow users to find devices 
 * that have been misplaced. Within the profile, two roles can be supported: Locator and Target.
 * The Find Me Target shall be a server. The Find Me Locator shall be a client.
 * When the Find Me Locator device wishes to cause an alert on the Find Me Target device, 
 * it shall write the specific Alert Level in the Alert Level characteristic.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_FINDME_TARGET
#include "findt.h"
#include "findt_task.h"
#include "app_findt_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create find me target database   
 *
 ****************************************************************************************
 */
void app_findt_create_db(void);

/*
 ****************************************************************************************
 * @brief Enable find me target service  
 *
 ****************************************************************************************
 */
void app_findt_enable_req(uint16_t conhdl, uint8_t sec_lvl);

#endif // BLE_FINDME_TARGET

/// @} APP_FINDT_API

#endif // APP_FINDT_H_
