/**
 ****************************************************************************************
 *
 * @file app_scppc.h
 *
 * @brief Application SCPPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_SCPPC_H_
#define APP_SCPPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_SCPP Scan Parameters Profile API
 * @ingroup APP
 * @brief Scan Parameters Profile API
 *
 *  The Scan Parameters Profile is used to provide devices with information to assist them in managed their connection 
 *  idle timeout and advertising parameters to optimize for power consumption and/or reconnection latency. 
 *  This service has been implemented as a profile. Within this profile, two roles can be supported: Server role (SCPPS) 
 *  and Client role (SCPPC). The Client role must support the GAP Central Role and the Server role, the GAP Peripheral 
 *  role. The profile requires a connection to be established between the two devices for its functionality. 
 *  The various documents edited by the Bluetooth SIG present different use cases for this profile, their GATT, GAP and 
 *  security, mandatory and optional requirements. The Scan Parameters Profile specifications have been adopted by the 
 *  Bluetooth SIG on December 27th 2011 ([1] and [3]). Their related Test Specifications have been released at the same 
 *  time and are referenced in [2] and [4]. 
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each task has an API decided 
 *  after the study of the profile specifications and test specifications, and it is considered to be minimalistic and designed 
 *  for a future application which would combine the profile functionality with the device connectivity and security 
 *  procedures. 
 *  The Scan Interval Window Characteristic is used to store the scan parameters of the Client. This parameters are the 
 *  last known values of LE_Scan Interval (Maximum Scan Interval the Scan Client intends to use while scanning) and 
 *  LE_Scan Window (Minimum Scan Windows the Scan Client intends to use while scanning in conjunction with the 
 *  maximum Scan Interval written) of the Client. 
 *  The Scan Refresh characteristic is used to nitofy the Client that the Server requires the Scan Interval Window 
 *  characteristic to be written with the latest values upon notification. 
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_SCPPC_API Scan Parameters Profile Client API
 * @ingroup APP_SCPPC
 * @brief Scan Parameters Profile Client API
 *
 *  Scan Parameters Profile Client APIs are used by APP to enable/disable Scan Parameters Profile Client Role,  
 *  to read/write Scan Refresh Notification Configuration Value.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_SP_CLIENT
#include "scppc.h"
#include "scppc_task.h"
#include "app_scppc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Scan Parameters profile Client Role - at connection
 *
 ****************************************************************************************
 */
void app_scppc_enable_req(uint16_t scan_intv, uint16_t scan_window, struct scps_content *scps, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to Write Scan Interval Window Characteristic Value Request 
 * - Write Without Response
 *
 ****************************************************************************************
 */
void app_scppc_scan_intv_wd_wr_req(uint16_t scan_intv, uint16_t scan_window, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to Read Scan Refresh Notification Configuration Value request
 *
 ****************************************************************************************
 */
void app_scppc_scan_refresh_ntf_cfg_rd_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to Write Scan Refresh Notification Configuration Value request
 *
 ****************************************************************************************
 */
void app_scppc_wr_meas_intv_req(uint16_t ntf_cfg, uint16_t conhdl);

#endif /* BLE_SP_CLIENT */

/// @} APP_SCPPC_API

#endif // APP_SCPPC_H_
