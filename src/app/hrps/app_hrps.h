/**
 ****************************************************************************************
 *
 * @file app_hrps.h
 *
 * @brief Application HRPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_HRPS_H_
#define APP_HRPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HRPS_API Heart Rate Profile Sensor
 * @ingroup APP_HRPS
 * @brief Heart Rate Profile Sensor
 *
 * The Bluetooth Low Energy Heart Rate profile enables the user to receive Heart Rate 
 * measurements from a Heart Rate sensor device and also configure it for different use cases.
 * Within the profile, two roles can be supported: Collector and Sensor. The Heart Rate Sensor
 * shall be a Server. The Collector shall be a Client.
 * 
 * Heart Rate Profile Sensor(HRPS): A HRPS (e.g. PC, phone, etc)
 * is the term used by this profile to describe a device that can perform Heart Rate
 * measurement and notify about on-going measurement and indicate final result to a peer
 * BLE device.
 *
 * Application needs manages multiple users configuration and storage of offline measurements.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_HR_SENSOR
#include "hrps.h"
#include "hrps_task.h"
#include "app_hrps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the heart rate service database - at initiation     
 *
 ****************************************************************************************
 */
void app_hrps_create_db(uint8_t features);

/*
 ****************************************************************************************
 * @brief Start the heart rate profile - at connection    
 * 
 ****************************************************************************************
 */
void app_hrps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type,
                         uint16_t hr_meas_ntf_en, uint8_t body_sensor_loc);

/*
 ****************************************************************************************
 * @brief Send Heart Rate measurement value - at connection 
 *
 ****************************************************************************************
 */
void app_hrps_measurement_send(uint16_t conhdl, struct hrs_hr_meas *meas_val);

#endif // BLE_HR_SENSOR

/// @} APP_HRPS_API

#endif // APP_HRPS_H_
