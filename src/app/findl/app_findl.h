/**
 ****************************************************************************************
 *
 * @file app_findl.h
 *
 * @brief Application FMPL API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_FINDL_H_
#define APP_FINDL_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_FIND Find Me Profile API
 * @ingroup APP
 * @brief Find Me Profile API
 *
 *  The Bluetooth Low Energy Find Me profile enables the user to locate a device using another 
 *  device connected to it. Within the profile, two roles can be supported: Locator and Target. 
 *  The Locator must support the GAP Central Role and the Target, the GAP Peripheral role. The 
 *  profile requires a connection to be established between the two devices for its functionality.
 *  The functionality of a profile requires the presence of certain services and attributes on one of 
 *  the two devices, which the other device can manipulate. In this case, the Find Me Target 
 *  device must have an instance of the Immediate Alert Service(IAS) in its attribute database. The 
 *  Find Me Locator(FINDL) will discover this service and its Alert Level Characteristic, which it may 
 *  then write to one of the three possible alert levels(None/Mild/High) to cause the Target(FINDT) 
 *  device to alert or stop alerting once it has been located.
 *  The various documents edited by the Bluetooth SIG PUID Working group present different use 
 *  cases for this profile, their GATT, GAP and security,  mandatory and optional  requirements. 
 *  The FMP profile and IAS Service specifications have been adopted by the Bluetooth SIG on June 
 *  21st 2011 ([1], [2]). Their related Test Specifications have been released at the same time and 
 *  are referenced in [3], [4].
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
 * @addtogroup APP_FINDL_API Find Me Locator API
 * @ingroup APP_FINDL
 * @brief Find Me Profile Locator API
 *
 *  FINDL role is meant to be activated on the device that will locate the Target. It implies it is a GAP 
 *  Central. The FW task for this role will discover the Immediate Alert Service present on the peer 
 *  Server, after establishing connection, and will allow writing different alert levels to the Alert 
 *  Level characteristic in the IAS. This file contain the implementation of this API
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_FINDME_LOCATOR
#include "findl.h"
#include "findl_task.h"
#include "app_findl_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Find Me Locator profile - at connection
 *
 ****************************************************************************************
 */
void app_findl_enable_req(struct ias_content *ias, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Alert level set request
 *
 ****************************************************************************************
 */
void app_findl_set_alert_req(uint8_t alert_lvl, uint16_t conhdl);

#endif /* BLE_FINDME_LOCATOR */

/// @} APP_FINDL_API

#endif // APP_FINDL_H_
