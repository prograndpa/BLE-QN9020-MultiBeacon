/**
 ****************************************************************************************
 *
 * @file app_diss.h
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

#ifndef APP_DISS_H_
#define APP_DISS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_DISS_API Device Information Service Server
 * @ingroup APP_DISS
 * @brief Device Information Service Server
 *
 * The Bluetooth Low Energy Device Information Service enables the user to expose
 * manufacturer and/or vendor information about a device.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_DIS_SERVER)
#include "diss.h"
#include "diss_task.h"
#include "app_diss_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create device information service database   
 * 
 ****************************************************************************************
 */
void app_diss_create_db(uint16_t features);

/*
 ****************************************************************************************
 * @brief Enable device information service   
 *
 ****************************************************************************************
 */
void app_diss_enable_req(uint16_t conhdl, uint8_t sec_lvl);

/*
 ****************************************************************************************
 * @brief Set Characteristic value
 *
 ****************************************************************************************
 */
void app_set_char_val_req(uint8_t char_code, uint8_t val_len, uint8_t const *val);

#endif // BLE_DIS_SERVER

/// @} APP_DISS_API

#endif // APP_DISS_H_
