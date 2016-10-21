/**
 ****************************************************************************************
 *
 * @file app_scpps.h
 *
 * @brief Application SCPPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_SCPPS_H_
#define APP_SCPPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_SCPPS_API Scan Parameters Profile Server
 * @ingroup APP_SCPPS
 * @brief Scan Parameters Profile Server
 *
 * The Bluetooth Low Energy Scan Parameters Profile is used to provide devices with information
 * to assist them in managing their connection idle timeout and advertising parameters to optimize
 * for power consumption and/or reconnection latency.
 * Within the profile, two roles can be supported: Client and Server. 
 * The Scan Server shall be a server. The Scan Client shall be a client.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_SP_SERVER
#include "scpps.h"
#include "scpps_task.h"
#include "app_scpps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the scan parameters profile server database - at initiation  
 * 
 ****************************************************************************************
 */
void app_scpps_create_db(uint8_t features);

/*
 ****************************************************************************************
 * @brief Start the scan parameters profile server service - at connection   
 *
 ****************************************************************************************
 */
void app_scpps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t scan_refresh_ntf_en);

/*
 ****************************************************************************************
 * @brief Send the scan refresh value - at connection  
 *
 ****************************************************************************************
 */
void app_scpps_scan_refresh_req(uint16_t conhdl, uint8_t scan_refresh); 

#endif // BLE_SP_SERVER

/// @} APP_SCPPS_API

#endif // APP_SCPPS_H_
