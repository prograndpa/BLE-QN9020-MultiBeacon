/**
 ****************************************************************************************
 *
 * @file app_blpc.h
 *
 * @brief Application BLPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BLPC_H_
#define APP_BLPC_H_

/// @cond
/**
 ****************************************************************************************
 * @addtogroup APP_BLP Blood Pressure Profile API
 * @ingroup APP
 * @brief Blood Pressure Profile API
 *
 *  The Bluetooth Low Energy Blood Pressure profile enables the user to receive blood pressure measurements from a 
 *  blood pressure sensor device and also configure it for different use cases. Within the profile, two roles can be 
 *  supported: Collector and Sensor. The Collector must support the GAP Central Role and the Sensor, the GAP Peripheral 
 *  role. The profile requires a connection to be established between the two devices for its functionality. 
 *  The functionality of a profile requires the presence of certain services and attributes on one of the two devices, which 
 *  the other device can manipulate. In this case, the Blood Pressure device must have one instance of the Blood Pressure 
 *  Service(BPS) and one instance of Device Information Service(DIS) in its attribute database. The Blood Pressure Profile 
 *  Collector (BLPC) will discover these services and their characteristics, and it may then configure them to cause the 
 *  Blood Pressure Profile Sensor (BLPS) device to take measurements and indicate/notify them to the Collector. 
 *  The various documents edited by the Bluetooth SIG Medical Working group present different use cases for this 
 *  profile, their GATT, GAP and security, mandatory and optional requirements. The BLP profile and BPS, DIS services 
 *  specifications have been adopted by the Bluetooth SIG on October 25th 2011 ([1], [2], [3]). Their related Test 
 *  Specifications have been released at the same time and are referenced in [4], [5], [6]. 
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each task has an API decided 
 *  after the study of the profile specifications and test specifications, and it is considered to be minimalistic and designed 
 *  for a future application which would combine the profile functionality with the device connectivity and security 
 *  procedures.
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_BLPC_API Blood Pressure Profile Collector API
 * @ingroup APP_BLPC
 * @brief Blood Pressure Profile Collector API
 *
 *  BLPC role is meant to be activated on the device that will collect the blood pressure measurements from the Blood 
 *  Pressure Sensor. It implies it is a GAP Central. The FW task for this role will discover the BPS and DIS present on the 
 *  peer Server, after establishing connection, and will allow configuration of the BPS attributes if so required. This 
 *  file contain the implementation of this API. 
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_BP_COLLECTOR
#include "blpc.h"
#include "blpc_task.h"
#include "app_blpc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Blood Pressure Collector profile - at connection
 *
 ****************************************************************************************
 */
void app_blpc_enable_req(struct bps_content *bps, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a BPS characteristic value
 *
 ****************************************************************************************
 */
void app_blpc_rd_char_req(uint8_t char_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 2 characteristics that can be handled
 *
 ****************************************************************************************
 */
void app_blpc_cfg_indntf_req(uint8_t char_code, uint16_t cfg_val, uint16_t conhdl);

#endif /* BLE_BP_COLLECTOR */

/// @} APP_BLPC_API

#endif // APP_BLPC_H_
