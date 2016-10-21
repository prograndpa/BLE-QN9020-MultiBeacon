/**
 ****************************************************************************************
 *
 * @file app_glps.h
 *
 * @brief Application GLPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef APP_GLPS_H_
#define APP_GLPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_GLPS_API Glucose Profile Sensor
 * @ingroup APP_GLPS
 * @brief Glucose Profile Sensor
 *
 * The Bluetooth Low Energy Glucose profile enables the user to manage measurements from 
 * a Glucose sensor device and also configure it for different use cases. Within the profile,
 * two roles can be supported: Collector and Sensor.
 * The Glucose Sensor shall be a Server. The Collector shall be a Client.
 *
 * The functionality of the profile requires the presence of certain services and attributes
 * on one of the two devices, which the other device can manipulate. In this case, the Glucose
 * device must have one instance of the Glucose Service(GLS) and one instance of Device Information
 * Service(DIS) in its attribute database. The Glucose Profile Collector (GLPC) will discover
 * these services and their characteristics, and it may then configure them to cause the 
 * Glucose Profile Sensor (GLPS) device to take measurements and notify them to the Collector.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_GL_SENSOR
#include "glps.h"
#include "glps_task.h"
#include "app_glps_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/*
 ****************************************************************************************
 * @brief Create the glucose service database - at initiation     
 *
 ****************************************************************************************
 */
void app_glps_create_db(uint16_t start_hdl, uint8_t meas_ctx_supported);

/*
 ****************************************************************************************
 * @brief Start the glucose profile - at connection     
 * 
 ****************************************************************************************
 */
void app_glps_enable_req(uint16_t conhdl, uint16_t features, uint8_t sec_lvl,
                         uint8_t con_type, uint16_t evt_cfg);

/*
 ****************************************************************************************
 * @brief Send Record Access Control Point (RACP) request response - at connection 
 *
 ****************************************************************************************
 */
void app_glps_racp_rsp_req_send(uint16_t conhdl, uint16_t num_of_record, uint8_t op_code, uint8_t status);

/*
 ****************************************************************************************
 * @brief Send glucose measurement without following measurement context information 
 * 
 ****************************************************************************************
 */
void app_glps_meas_without_ctx_req_send(uint16_t conhdl, uint16_t seq_num, struct glp_meas *meas);

/*
 ****************************************************************************************
 * @brief Send glucose measurement with following measurement context information  
 *
 ****************************************************************************************
 */
void app_glps_meas_with_ctx_req_send(uint16_t conhdl, uint16_t seq_num, struct glp_meas *meas, struct glp_meas_ctx *ctx);

#endif // BLE_GL_SENSOR

/// @} APP_GLPS_API

#endif // APP_GLPS_H_
