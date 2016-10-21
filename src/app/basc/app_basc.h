/**
 ****************************************************************************************
 *
 * @file app_basc.h
 *
 * @brief Application BASC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BASC_H_
#define APP_BASC_H_


///@cond
/*
 ****************************************************************************************
 * @addtogroup APP_BAS Battery Service API
 * @ingroup APP
 * @brief Battery Service API
 *
 *  The BLE BAS enables a device to expose its current battery level to a peer device. 
 *  This service has been implemented as a profile. Within this profile, two roles can be supported: Server role (BASS) and 
 *  Client role (BASC). The Client role must support the GAP Central Role and the Server role, the GAP Peripheral role. The 
 *  profile requires a connection to be established between the two devices for its functionality.  
 *  The various documents edited by the Bluetooth SIG present different use cases for this profile, their GATT, GAP and 
 *  security, mandatory and optional requirements. The Battery Service (BAS) specification has been adopted by the 
 *  Bluetooth SIG on December 27th 2011 ([1]). Its related Test Specification have been released at the same time and are 
 *  referenced in [2].  
 *  The profile is implemented in the BLE software stack as two tasks, one for each role. Each task has an API decided 
 *  after the study of the profile specifications and test specifications, and it is considered to be minimalistic and designed 
 *  for a future application which would combine the profile functionality with the device connectivity and security procedures. 
 *
 *****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup APP_BASC Battery Service Client
 * @ingroup APP_BAS
 * @brief Battery Service Client
 *
 *  The BLE BAS Client role has been designed to allow a collector to be inform about the current 
 *  battery level of a device either upon the reception of notifications or upon a read request. 
 *   
 ****************************************************************************************
 */
///@endcond

/**
 ****************************************************************************************
 * @addtogroup APP_BASC_API Battery Service Client API
 * @ingroup APP_BASC
 * @brief Battery Service Client API
 *
 *  Battery Service Client APIs are used by APP to enable/disable the Battery Service Client Role,  
 *  to read the value of a characteristic, or to write battery level notification configuration.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

#if BLE_BATT_CLIENT
#include "basc.h"
#include "basc_task.h"
#include "app_basc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Battery Service Client Role - at connection
 *
 ****************************************************************************************
 */
void app_basc_enable_req(uint8_t bas_nb, struct bas_content *bas, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Read Characteristic Value Request
 *
 ****************************************************************************************
 */
void app_basc_rd_char_req(uint8_t char_code, uint8_t bas_nb, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write Battery Level Notification Configuration Value request
 *
 ****************************************************************************************
 */
void app_basc_cfg_indntf_req(uint16_t ntf_cfg, uint8_t bas_nb, uint16_t conhdl);

#endif /* BLE_BATT_CLIENT */

/// @} APP_BASC_API

#endif // APP_BASC_H_
