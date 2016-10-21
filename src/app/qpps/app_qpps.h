/**
 ****************************************************************************************
 *
 * @file app_qpps.h
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
 
#ifndef APP_QPPS_H_
#define APP_QPPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_QPPS_API Quintic Private Profile Server
 * @ingroup APP_QPPS
 * @brief Quintic Private Profile Server
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_QPP_SERVER
#include "qpps_task.h"
#include "app_qpps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create Quintic Private Profile service database - at initiation
 *
 ****************************************************************************************
 */
void app_qpps_create_db(uint8_t char_num);

/*
 ****************************************************************************************
 * @brief Start the profile - at connection    
 * 
 ****************************************************************************************
 */
void app_qpps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t ntf_en);

/*
 ****************************************************************************************
 * @brief Send a notification containing raw data - at connection 
 *
 ****************************************************************************************
 */
void app_qpps_data_send(uint16_t conhdl, uint8_t index, uint8_t length, uint8_t *data);

#endif // BLE_QPP_SERVER

/// @} APP_QPPS_API

#endif // APP_QPPS_H_
