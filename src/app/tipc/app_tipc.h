/**
 ****************************************************************************************
 *
 * @file app_tipc.h
 *
 * @brief Application TIPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_TIPC_H_
#define APP_TIPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_TIP Time Profile API
 * @ingroup APP
 * @brief Time Profile API
 *
 *  The Bluetooth Low Energy Time profile enables the user to obtain the current date and time, 
 *  and related information such as time zone as exposed by the Current Time service in the peer 
 *  device. Information of when next change of daylight savings time (DST) will occur can be 
 *  retrieved from the peer exposed by the Next DST Change service. This profile also enables a 
 *  device to request updating the time on the peer device as exposed by the Reference Time 
 *  Update Service.
 *  Within the profile, two roles can be supported: Client and Server. The Client must support the 
 *  GAP Peripheral Role and the Server, the GAP Central role. The profile requires a connection to 
 *  be established between the two devices for its functionality. 
 *  The functionality of a profile requires the presence of certain services and attributes on one of 
 *  the two devices, which the other device can manipulate. In this case, the Time Server device 
 *  must have one instance of the Current Time Service (CTS) in its attribute database; the Next 
 *  DST Change Service (NDCS) and the Reference Time Update Service (RTUS) are optional. The 
 *  Time Profile Client (TIPC) will discover these services and their characteristics, and it may then 
 *  configure them to cause the Time Server (TIPS) to notify the Current Time value to the Client or 
 *  to require a time update. 
 *  The various documents edited by the Bluetooth SIG present different use cases for this profile, 
 *  their GATT, GAP and security, mandatory and optional requirements. The TIP profile and CTS, 
 *  NDCS, RTUS services specifications have been adopted by the Bluetooth SIG on September 15th 
 *  2011 ([1], [2], [3], [4]). Their related Test Specifications have been released at the same time 
 *  and are referenced in [5], [6], [7] and [8]. 
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each 
 *  task has an API decided after the study of the profile specifications and test specifications, and 
 *  it is considered to be minimalistic and designed for a future application which would combine 
 *  the profile functionality with the device connectivity and security procedures. 
 *
 *****************************************************************************************
 */
/// @endcond
 
/**
 ****************************************************************************************
 * @addtogroup APP_TIPC_API Time Profile Client API
 * @ingroup APP_TIPC
 * @brief Time Profile Client API
 *
 *  TIPC role is meant to be activated on the device that will collect the time values and 
 *  information from the Time Server. It implies it is a GAP Central. The FW task for this role will 
 *  discover the CTS (Mandatory), the NDCS (Optional) and the RTUS (Optional) present on the 
 *  peer Server, after establishing connection, and will allow configuration of the CTS and RTUS  
 *  attributes if so required. This file contain the implementation of this API.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

#if BLE_TIP_CLIENT
#include "tipc.h"
#include "tipc_task.h"
#include "app_tipc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Time profile Client role - at connection
 *
 ****************************************************************************************
 */
void app_tipc_enable_req(struct tipc_cts_content *cts,
                         struct tipc_ndcs_content *ndcs,
                         struct tipc_rtus_content *rtus,
                         uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read a CTS or NDCS or RTUS characteristic value
 *
 ****************************************************************************************
 */
void app_tipc_rd_char_req(uint8_t char_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for configuring the Current Time Characteristic on the Server
 *
 ****************************************************************************************
 */
void app_tipc_ct_ntf_cfg_req(uint16_t cfg_val, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message for writing the Time Update Control Point Characteristic Value 
 * on a peer device
 *
 ****************************************************************************************
 */
void app_tipc_wr_time_udp_ctnl_pt_req(uint8_t value, uint16_t conhdl);

#endif /* BLE_TIP_CLIENT */

/// @} APP_TIPC_API

#endif // APP_TIPC_H_
