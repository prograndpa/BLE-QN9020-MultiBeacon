/**
 ****************************************************************************************
 *
 * @file app_cscpc.h
 *
 * @brief Application CSCPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_CSCPC_H_
#define APP_CSCPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_CSCP Cycling Speed and Cadence Profile API
 * @ingroup APP
 * @brief Cycling Speed and Cadence Profile API
 *
 * The CSCP enables a collector device to connect and interact with a Cycling Speed and Cadence
 * Sensor for use in sport and fitness spplications.
 *
 * The CSC Collector is multi-instantiated, one instance is created for each connection for
 * which the profile will be enabled and each of these instances will hava a different task ID.
 * Thus, it is very important for the application to keep the source task ID of the first received
 * CSCPC_CMP_EVT message to be able to communicate with the peer device linked to this task ID 
 * once it has been enabled.
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_CSCPC_API Cycling Speed and Cadence Profile Client API
 * @ingroup APP_CSCPC
 * @brief Cycling Speed and Cadence Profile Client API
 *
 * Cycling Speed and Cadence Profile Client APIs are used by APP to enable/disable 
 * Cycling Speed and Cadence Profile Client Role, to read/write speed-related and cadence-related 
 * data from a Cycling Speed and Cadence sensor. The CSCPC task for this role will discover 
 * the CSCPS present on the peer Server, after establishing connection, and will allow configuration 
 * of the CSCPS attributes if so required. This file contain the implementation of this API. 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_CSC_COLLECTOR
#include "cscpc.h"
#include "cscpc_task.h"
#include "app_cscpc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Cycling Speed and Cadence profile Collector - at connection.
 ****************************************************************************************
 */
void app_cscpc_enable_req(struct cscpc_cscs_content *cscs, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Rread the value of an attribute in the peer device database.
 ****************************************************************************************
 */
void app_cscpc_read_req(uint8_t read_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Configure sending of notification/indication in the peer device database.
 *
 ****************************************************************************************
 */
void app_cscpc_cfg_ntfind_req(uint8_t desc_code, uint16_t ntfind_cfg, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Send the value of the SC Control Point.
 *
 ****************************************************************************************
 */
void app_cscpc_ctnl_pt_cfg_req(struct cscp_sc_ctnl_pt_req *sc_ctnl_pt, uint16_t conhdl);

#endif /* BLE_CSC_COLLECTOR */

/// @} APP_CSCPC_API

#endif // APP_CSCPC_H_
