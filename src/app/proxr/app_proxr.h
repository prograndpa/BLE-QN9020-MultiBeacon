/**
 ****************************************************************************************
 *
 * @file app_proxr.h
 *
 * @brief Application PROXR API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_PROXR_H_
#define APP_PROXR_H_

/**
 ****************************************************************************************
 * @addtogroup APP_PROXR_API Proximity Reporter
 * @ingroup APP_PROXR
 * @brief Proximity Profile Reporter
 *
 * The Proximity profile defines the behavior when a device moves away from a peer device
 * so that the connection is dropped or the path loss increases above a preset level, 
 * causing an immediate alert. This alert can be used to notify the user that the devices
 * have become separated. As a consequence of this alert, a device may take further action,
 * for example to lock one of the devices so that it is no longer usable.
 * The Proximity profile can also be used to define the behavior when the two devices come
 * closer together such that a connection is made or the path loss decreases below a preset
 * level.
 * Within the profile, two roles can be supported: Monitor and Reporter.
 * The Proximity Reporter shall be a server. The Proximity Monitor shall be a GATT client.
 *
 * The Proximity Reporter device must have an instance of the Link Loss Service(LLS), and 
 * may also have the Immediate Alert Service(IAS) and Tx Power Service(TPS) in its attribute
 * database. The two last ones must be used together, if one is missing, the other one 
 * should be ignored.
 *
 * The LLS allows the user to set an alert level in the Reporter, which will be used by the 
 * reporter to alert in the corresponding way if the link is lost. The disconnection must 
 * not come voluntarily from one of the two devices in order to trigger the alert.
 *
 * The IAS allows the user to set an immediate alert level based on path loss computation 
 * using the read Tx Power Level and RSSI monitored on received packets. According to the 
 * alert level set in IAS, the Reporter will start alerting immediately.
 *
 * The TPS allows the user to read the Tx Power Level for the physical layer. The value is 
 * used by the Monitor to continuously evaluate path loss during the connection, and decide 
 * to trigger/stop an alert based on path loss going over/under a set threshold in the Monitor
 * application.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_PROX_REPORTER
#include "proxr.h"
#include "proxr_task.h"
#include "app_proxr_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the proximity reporter service database - at initiation 
 *
 ****************************************************************************************
 */
void app_proxr_create_db(uint8_t features);

/*
 ****************************************************************************************
 * @brief Start the proximity reporter role - at connection  
 *
 ****************************************************************************************
 */
void app_proxr_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t lls_alert_lvl, int8_t txp_lvl);

#endif // BLE_PROX_REPORTER

/// @} APP_PROXR_API

#endif // APP_PROXR_H_
