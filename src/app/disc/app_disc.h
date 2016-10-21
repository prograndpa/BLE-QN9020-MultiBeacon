/**
 ****************************************************************************************
 *
 * @file app_disc.h
 *
 * @brief Application DISC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_DISC_H_
#define APP_DISC_H_

/// @cond
/**
 ****************************************************************************************
 * @addtogroup APP_DIS Device Information Service API
 * @ingroup APP
 * @brief Device Information Service API
 *
 *  The Bluetooth Low Energy Device Information Service enables the user to expose 
 *  manufacturer and/or vendor information about a device. Within the profile, two roles can be 
 *  supported: Client and Server. The client must support the GAP Central Role and the Server, 
 *  the GAP Peripheral role. The profile requires a connection to be established between the two 
 *  devices for its functionality.
 *  The various documents edited by the Bluetooth SIG GPA Working group present different use 
 *  cases for this profile, their GATT, GAP and security,  mandatory and optional  requirements. 
 *  The DIS specifications has been adopted by the Bluetooth SIG on November 19st 2011([1]). Its 
 *  related Test Specifications has been released at the same time and are referenced in [2],.
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each 
 *  task has an API decided after the study of the service specification and test specification, and it 
 *  is considered to be minimalistic and designed for a future application which would combine the 
 *  profile functionality with the device connectivity and security procedures.
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_DISC_API Device Information Service Client API
 * @ingroup APP_DISC
 * @brief Device Information Service Client API
 *
 *  DISC role is meant to be activated on the device that will locate the Server. It implies it is a GAP 
 *  Central. The FW task for this role will discover the Device Information Service present on the 
 *  peer Server, after establishing connection, and will allow reading different information about 
 *  the device. 
 *  This file contains the implementation of this API.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_DIS_CLIENT
#include "disc.h"
#include "disc_task.h"
#include "app_disc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Device Information profile - at connection
 *
 ****************************************************************************************
 */
void app_disc_enable_req(struct disc_dis_content *dis, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a DIS characteristic value
 *
 ****************************************************************************************
 */
void app_disc_rd_char_req(uint8_t char_code, uint16_t conhdl);

#endif /* BLE_DIS_CLIENT */

/// @} APP_DISC_API

#endif // APP_DISC_H_
