/**
 ****************************************************************************************
 *
 * @file app_cscps.h
 *
 * @brief Application CSCPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#ifndef APP_CSCPS_H_
#define APP_CSCPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_CSCPS_API Cycling Speed and Cadence Profile Sensor
 * @ingroup APP_CSCPS
 * @brief Cycling Speed and Cadence Profile Sensor
 *
 * The Cycling Speed and Cadence Profile is used to enable a data collection device to 
 * obtain data from a Cycling Speed and Cadence Sensor (CSC Sensor) that exposes the Cycling
 * Speed and Cadence Service.
 * The CSC Sensor is the device that reports wheel revolution data and/or crank revolution data.
 *
 * @{
 ****************************************************************************************
 */

/// @} APP_CSCPS_API

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_CSC_SENSOR
#include "cscps.h"
#include "cscps_task.h"
#include "app_cscps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create one instance of the Cycling Speed and Cadence Service database - at initiation
 *
 ****************************************************************************************
 */
void app_cscps_create_db(uint16_t csc_feature, uint8_t sensor_loc_supp, uint8_t sensor_loc);

/*
 ****************************************************************************************
 * @brief Enable the CSCP Sensor role for the specified connection - at connection
 *
 ****************************************************************************************
 */
void app_cscps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t csc_meas_ntf_cfg, 
                          uint16_t sc_ctnl_pt_ntf_cfg, uint32_t wheel_rev);
                          
/*
 ****************************************************************************************
 * @brief Send the CSCP Sensor Measurement value - at connection
 *
 ****************************************************************************************
 */
void app_cscps_ntf_csc_meas_req(uint16_t conhdl, uint8_t flags, uint16_t cumul_crank_rev, uint16_t last_crank_evt_time, 
                                uint16_t last_wheel_evt_time, int16_t wheel_rev);

/*
 ****************************************************************************************
 * @brief Application response for the CSCPS_SC_CTNL_PT_REQ_IND message - at connection
 *
 ****************************************************************************************
 */
void app_cscps_sc_ctnl_pt_cfm(uint16_t conhdl, uint8_t status, union cscps_sc_ctnl_pt_cfm_value *value);

#endif // BLE_CSC_SENSOR

/// @} APP_CSCPS_API

#endif // APP_CSCPS_H_
