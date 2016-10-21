/**
 ****************************************************************************************
 *
 * @file app_blps.h
 *
 * @brief Application BLPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BLPS_H_
#define APP_BLPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_BLPS_API Blood Pressure Profile Sensor
 * @ingroup APP_BLPS
 * @brief Blood Pressure Profile Sensor
 *
 * Blood Pressure Profile Sensor(BLPS): A BLPS (e.g. PC, phone, etc)
 * is the term used by this profile to describe a device that can perform blood pressure
 * measurement and notify about on-going measurement and indicate final result to a peer
 * BLE device.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_BP_SENSOR)
#include "blps.h"              
#include "blps_task.h"                
#include "app_blps_task.h"    

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create blood pressure service database   
 *
 ****************************************************************************************
 */
void app_blps_create_db(uint8_t features);

/*
 ****************************************************************************************
 * @brief Enable blood pressure service     
 *
 ****************************************************************************************
 */
void app_blps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t bp_meas_ind_en,
                         uint16_t interm_cp_ntf_en, uint16_t bp_feature);

/*
 ****************************************************************************************
 * @brief Send blood pressure measurement value from APP 
 *
 ****************************************************************************************
 */
void app_blps_pressure_send_req(uint16_t conhdl, uint8_t flag_interm, struct bps_bp_meas *meas_val);

#endif // BLE_BP_SENSOR

/// @} APP_BLPS_API

#endif // APP_BLPS_H_
