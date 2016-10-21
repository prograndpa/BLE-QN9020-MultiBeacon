/**
 ****************************************************************************************
 *
 * @file app_hrpc.h
 *
 * @brief Application HRPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_HRPC_H_
#define APP_HRPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_HRP Heart Rate Profile API
 * @ingroup APP
 * @brief Heart Rate Profile API
 *
 *  The Bluetooth Low Energy Heart Rate profile enables the user to receive Heart Rate measurements from a Heart Rate 
 *  sensor device and also configure it for different use cases. Within the profile, two roles can be supported: Collector 
 *  and Sensor. The Collector must support the GAP Central Role and the Sensor, the GAP Peripheral role. The profile 
 *  requires a connection to be established between the two devices for its functionality. 
 *  The functionality of a profile requires the presence of certain services and attributes on one of the two devices, which 
 *  the other device can manipulate. In this case, the Heart Rate device must have one instance of the Heart Rate 
 *  Service(HRS) and one instance of Device Information Service(DIS) in its attribute database. The Heart Rate Profile 
 *  Collector (HRPC) will discover these services and their characteristics, and it may then configure them to cause the 
 *  Heart Rate Profile Sensor (HRPS) device to take measurements and indicate/notify them to the Collector. 
 *  The various documents edited by the Bluetooth SIG Medical Working group present different use cases for this 
 *  profile, their GATT, GAP and security, mandatory and optional requirements. The HRP profile and HRS, DIS services 
 *  specifications have been adopted by the Bluetooth SIG on October  25th 2011 ([1], [2], [3]). Their related Test 
 *  Specifications have been released at the same time and are referenced in [4], [5], [6]. 
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each task has an API decided 
 *  after the study of the profile specifications and test specifications, and it is considered to be minimalistic and designed 
 *  for a future application which would combine the profile functionality with the device connectivity and security 
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_HRPC_API Heart Rate Profile Collector API
 * @ingroup APP_HRPC
 * @brief Heart Rate Profile Collector API
 *
 *  HRPC role is meant to be activated on the device that will collect the Heart Rate measurements from the Heart Rate 
 *  Sensor. It implies it is a GAP Central. The FW task for this role will discover the HRS present on the peer Server, after 
 *  establishing connection, and will allow configuration of the HRS attributes if so required. 
 *  This file contain the implementation of this API. 
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HR_COLLECTOR
#include "hrpc.h"
#include "hrpc_task.h"
#include "app_hrpc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Heart Rate Collector profile - at connection
 *
 ****************************************************************************************
 */
void app_hrpc_enable_req(struct hrs_content *hrs, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a HRS characteristic value
 *
 ****************************************************************************************
 */
void app_hrpc_rd_char_req(uint8_t char_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 2 characteristics that can be handled
 *
 ****************************************************************************************
 */
void app_hrpc_cfg_indntf_req(uint16_t cfg_val, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief APP request for control point write (to reset Energy Expanded)
 *
 ****************************************************************************************
 */
void app_hrpc_wr_cntl_point_req(uint8_t val, uint16_t conhdl);

#endif /* BLE_HR_COLLECTOR */

/// @} APP_HRPC_API

#endif // APP_HRPC_H_
