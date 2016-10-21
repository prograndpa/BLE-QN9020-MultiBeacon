/**
 ****************************************************************************************
 *
 * @file app_proxm.h
 *
 * @brief Application PROXM API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_PROXM_H_
#define APP_PROXM_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_PROX Proximity Profile API
 * @ingroup APP
 * @brief Proximity Profile API
 *
 *  The Bluetooth Low Energy Proximity profile enables the user to monitor proximity/link loss 
 *  between two devices. Within the profile, two roles can be supported: Monitor and Reporter. 
 *  The Monitor must support the GAP Central Role and the Reporter, the GAP Peripheral role. The 
 *  profile requires a connection to be established between the two devices for its functionality.
 *  The functionality of a profile requires the presence of certain services and attributes on one of 
 *  the two devices, which the other device can manipulate. In this case, the Proximity Reporter 
 *  device must have an instance of the Link Loss Service(LLS), and may also have the Immediate 
 *  Alert Service(IAS) and Tx Power Service(TPS) in its attribute database. The two last ones must 
 *  be used together, if one is missing, the other one should be ignored. The Proximity Monitor will 
 *  discover these services and their Characteristics: the Alert Level Characteristic in LLS, the Alert 
 *  Level Characteristic in IAS and the Tx Power Level Characteristic in TPS.
 *  The LLS allows the user to set an alert level in the Reporter, which will be used by the reporter 
 *  to alert in the corresponding way if the link is lost. The disconnection must not come 
 *  voluntarily from one of the two devices in order to trigger the alert.
 *  The IAS allows the user to set an immediate alert level based on path loss computation using 
 *  the read Tx Power Level and RSSI monitored on received packets. According to the alert level 
 *  set in IAS, the Reporter will start alerting immediately. 
 *  The TPS allows the user to read the Tx Power Level for the physical layer. The value is used by 
 *  the Monitor to continuously evaluate path loss during the connection, and decide to 
 *  trigger/stop an alert based on path loss going over/under a set threshold in the Monitor 
 *  application.
 *  The security requirements by this profile are enforced through the Attribute properties, and by 
 *  the Application: the Application on the Reporter device should start security procedure prior to 
 *  enabling the Reporter role, just as the Application on the Monitor device should enable the 
 *  Monitor role after the security procedure is over. The user normally has knowledge of the 
 *  security requirements of both devices so a confusion is hard to take place.
 *  The various documents edited by the Bluetooth SIG PUID Working group present different use 
 *  cases for this profile, their GATT, GAP and security,  mandatory and optional  requirements. 
 *  The PXP profile and IAS, LLS, TPS Service specifications have been adopted by the Bluetooth 
 *  SIG on June 21st 2011 ([1], [2], [3], [4]). Their related Test Specifications have been released at 
 *  the same time and are referenced in [5], [6], [7], [8].
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
 * @addtogroup APP_PROXM_API Proximity Monitor API
 * @ingroup APP_PROXM
 * @brief Proximity Profile Monitor API
 *
 *  PEOXM role is meant to be activated on the device that will monitor the connection with the 
 *  Reporter device. It implies it is a GAP Central. The FW task for this role will discover the LLS, 
 *  IAS and TPS services present on the peer Server, after establishing connection, and will allow 
 *  writing different alert levels to the Alert Level characteristic in the LLS or IAS, and also reading 
 *  the Tx Power Level characteristic in TPS. 
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_PROX_MONITOR
#include "proxm.h"
#include "proxm_task.h"
#include "app_proxm_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Proximity Monitor profile - at connection
 *
 ****************************************************************************************
 */
void app_proxm_enable_req(struct svc_content *lls, 
                          struct svc_content *ias,
                          struct svc_content *txps,
                          uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to Read LLS Alert level (for IAS  not allowed)
 *
 ****************************************************************************************
 */
void app_proxm_rd_alert_lvl_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to Set Alert level
 *
 ****************************************************************************************
 */
void app_proxm_wr_alert_lvl_req(uint8_t svc_code, uint8_t lvl, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to Read TX Power Level
 *
 ****************************************************************************************
 */
void app_proxm_rd_txpw_lvl_req(uint16_t conhdl);

#endif /* BLE_PROX_MONITOR */

/// @} APP_PROXM

#endif // APP_PROXM_H_
