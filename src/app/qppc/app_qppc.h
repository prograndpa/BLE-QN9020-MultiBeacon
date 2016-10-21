/**
 ****************************************************************************************
 *
 * @file app_qppc.h
 *
 * @brief Application QPPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef APP_QPPC_H_
#define APP_QPPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_QPP Quintic Private Profile API
 * @ingroup APP
 * @brief Quintic Private Profile API
 *
 *  The Bluetooth Low Energy Quintic Private profile enables the user to send or receive raw data
 *  from a Quintic Private Profile server device and also configure it for different use cases.
 *  Within the profile, two roles can be supported: Client and Server.
 *  The Client must support the GAP Central Role and the Server, the GAP Peripheral role. 
 *  The profile requires a connection to be established between the two devices for its functionality. 
 *  The Server must have one instance of the Service(QPPS) in its attribute database. The Client (QPPC)  
 *  will discover the service and their characteristics, and it may then configure them to cause the 
 *  Server (QPPS) device to send notify to the Client. 
 *  The various documents edited by the Bluetooth SIG Medical Working group present different use cases 
 *  for this profile, their GATT, GAP and security, mandatory and optional requirements. 
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each task has 
 *  an API decided after the study of the profile specification.
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_QPPC_API Quintic Private Profile Client API
 * @ingroup APP_QPPC
 * @brief Quintic Private Profile Client API
 *
 *  QPPC role is meant to be activated on the device that will collect the characteristic from the
 *  Server. It implies it is a GAP Central. The FW task for this role will discover the QPPS present
 *  on the peer Server, after establishing connection, and will allow configuration of the QPPS attributes
 *  if so required. 
 *  This file contain the implementation of this API.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_QPP_CLIENT
#include "qppc_task.h"
#include "app_qppc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the profile - at connection
 *
 ****************************************************************************************
 */
void app_qppc_enable_req(struct qpps_content *hrs, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a QPPS characteristic descriptor values
 *
 ****************************************************************************************
 */
void app_qppc_rd_char_req(uint16_t conhdl, uint8_t char_code);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the characteristics that can be handled
 *
 ****************************************************************************************
 */
void app_qppc_cfg_indntf_req(uint16_t cfg_val, uint16_t conhdl, uint8_t char_code);

/*
 ****************************************************************************************
 * @brief Send data to server
 *
 ****************************************************************************************
 */
void app_qppc_wr_data_req(uint8_t len, uint8_t *val, uint16_t conhdl);

#endif /* BLE_QPP_CLIENT */

/// @} APP_QPPC_API

#endif // APP_QPPC_H_
