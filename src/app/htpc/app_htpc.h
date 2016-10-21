/**
 ****************************************************************************************
 *
 * @file app_htpc.h
 *
 * @brief Application HTPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_HTPC_H_
#define APP_HTPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_HTP Health Thermometer Profile API
 * @ingroup APP
 * @brief Health Thermometer Profile API
 *
 *  The Bluetooth Low Energy Health Thermometer profile enables the user to receive 
 *  temperature measurements from a thermometer device and also configure it for different use 
 *  cases. Within the profile, two roles can be supported:  Collector  and  Thermometer. The 
 *  Collector must support the GAP Central Role and the Thermometer, the GAP Peripheral role. 
 *  The profile requires a connection to be established between the two devices for its 
 *  functionality.
 *  The functionality of a profile requires the presence of certain services and attributes on one of 
 *  the two devices, which the other device can manipulate. In this case, the Thermometer device 
 *  must have one instance of the Health Thermometer Service (HTS) and one instance of Device 
 *  Information Service (DIS) in its attribute database. The DIS shall be manage separately using 
 *  the DISS task (please refer to the BLE Device Information Service Interface Specification). 
 *  The Health Thermometer Collector (HTPC) will discover these services and their characteristics, 
 *  and it may then configure them to cause the Thermometer (HTPT) device to take 
 *  measurements and indicate/notify them to the Collector.
 *  The various documents edited by the Bluetooth SIG Medical Working group present different 
 *  use cases for this profile, their GATT, GAP and security, mandatory and optional  requirements. 
 *  The HTP profile and HTS specifications have been adopted by the Bluetooth SIG on May 24th 
 *  2011 ([1], [2]). Their related Test Specifications have been released at the same time and are 
 *  referenced in [3], [4].
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each 
 *  task has an API decided after the study of the profile specifications and test specifications, and 
 *  it is considered to be minimalistic and designed for a future application which would combine 
 *  the profile functionality with the device connectivity and security procedures
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_HTPC_API Health Thermometer Profile Collector API
 * @ingroup APP_HTPC
 * @brief Health Thermometer Profile Collector API
 *
 * HTPC role is meant to be activated on the device that will collect the temperature 
 * measurements from the Thermometer. It implies it is a GAP Central. The FW task for this role 
 * will discover the HTS present on the peer Server, after establishing connection, and will allow 
 * configuration of the HTS attributes if so required. This file contains the implementation of this API.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_HT_COLLECTOR
#include "htpc.h"
#include "htpc_task.h"
#include "app_htpc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Health Thormometer Collector profile - at connection
 *
 ****************************************************************************************
 */
void app_htpc_enable_req(struct htpc_hts_content *hts, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a HTs characteristic value
 *
 ****************************************************************************************
 */
void app_htpc_rd_char_req(uint8_t char_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 3 characteristics that can be handled
 *
 ****************************************************************************************
 */
void app_htpc_cfg_indntf_req(uint8_t char_code, uint16_t cfg_val, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief APP request for measurement interval write
 *
 ****************************************************************************************
 */
void app_htpc_wr_meas_intv_req(uint16_t intv, uint16_t conhdl);

#endif /* BLE_HT_COLLECTOR */

/// @} APP_HTPC_API

#endif // APP_HTPC_H_
