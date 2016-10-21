/**
 ****************************************************************************************
 *
 * @file app_rscps.h
 *
 * @brief Application RSCPS API
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

#ifndef APP_RSCPS_H_
#define APP_RSCPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_RSCPS_API Running Speed and Cadence Profile Sensor
 * @ingroup APP_RSCPS
 * @brief Running Speed and Cadence Profile Sensor
 *
 * The Running Speed and Cadence Profile is used to enable a data collection device to obtain data
 * from a Running Speed and Cadence Sensor (RSC Sensor) that exposes the Running Speed and Cadence Service.
 * The RSC Sensor is the device that reports speed-related data and/or cadence-related data.
 *
 * @{
 ****************************************************************************************
 */

/// @} APP_RSCPS_API

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_RSC_SENSOR
#include "rscps.h"
#include "rscps_task.h"
#include "app_rscps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Create one instance of the Running Speed and Cadence Service database - at initiation
 *
 ****************************************************************************************
 */
void app_rscps_create_db(uint16_t rsc_feature, uint8_t sensor_loc_supp, uint8_t sensor_loc);

/*
 ****************************************************************************************
 * @brief Enable the RSCP Sensor role for the specified connection - at connection
 *
 ****************************************************************************************
 */
void app_rscps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t rsc_meas_ntf_cfg, 
                          uint16_t sc_ctnl_pt_ntf_cfg);

/*
 ****************************************************************************************
 * @brief Send the RSC Sensor Measurement value - at connection
 *
 ****************************************************************************************
 */
void app_rscps_ntf_rsc_meas_req(uint16_t conhdl, uint8_t flags, uint8_t inst_cad, uint16_t inst_speed, 
                                uint16_t inst_stride_len, uint32_t total_dist);
                                
/*
 ****************************************************************************************
 * @brief Send the RSC Sensor Measurement value - at connection
 *
 ****************************************************************************************
 */
void app_rscps_ntf_rsc_meas_req(uint16_t conhdl, uint8_t flags, uint8_t inst_cad, uint16_t inst_speed, 
                                uint16_t inst_stride_len, uint32_t total_dist);

/*
 ****************************************************************************************
 * @brief Application response for the RSCPS_SC_CTNL_PT_REQ_IND message - at connection
 *
 ****************************************************************************************
 */
void app_rscps_sc_ctnl_pt_cfm(uint16_t conhdl, uint8_t status, union rscps_sc_ctnl_pt_cfm_value *value);

#endif // BLE_RSC_SENSOR

/// @} APP_RSCPS_API

#endif // APP_RSCPS_H_
