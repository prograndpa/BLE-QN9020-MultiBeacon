/**
 ****************************************************************************************
 *
 * @file app_glpc.h
 *
 * @brief Application GLPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_GLPC_H_
#define APP_GLPC_H_

/**
 ****************************************************************************************
 * @addtogroup APP_GLPC_API Glucose Profile Collector API
 * @ingroup APP_GLPC
 * @brief Glucose Profile Collector API
 *
 *  GLPC role is meant to be activated on the device that will collect the Glucose measurements from the Glucose sensor. 
 *  It implies it is a GAP Central. The FW task for this role will discover the GLS present on the peer Server, after 
 *  establishing connection, and will allow configuration of the GLS attributes if so required. 
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_GL_COLLECTOR
#include "glpc.h"
#include "glpc_task.h"
#include "app_glpc_task.h"


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Glucose Profile Collector - at connection
 *
 ****************************************************************************************
 */
void app_glpc_enable_req(struct gls_content *gls, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Register to Glucose sensor notifications - request
 *
 ****************************************************************************************
 */
void app_glpc_register_req(bool meas_ctx_en, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to read Glucose sensor features - request
 *
 ****************************************************************************************
 */
void app_glpc_read_features_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to record Access Control Point - Request
 *
 ****************************************************************************************
 */
void app_glpc_racp_req(struct glp_racp_req *racp, uint16_t conhdl);

#endif /* BLE_GL_COLLECTOR */

/// @} APP_GLPC_API

#endif // APP_GLPC_H_
