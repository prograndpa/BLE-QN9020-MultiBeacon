/**
 ****************************************************************************************
 *
 * @file app_rscpc.h
 *
 * @brief Application RSCPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_RSCPC_H_
#define APP_RSCPC_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup APP_RSCP Running Speed and Cadence Profile API
 * @ingroup APP
 * @brief Running Speed and Cadence Profile API
 *
 * The Running Speed and Cadence Profile is used to enable a data collection device to 
 * obtain data from a Running Speed and Cadence Sensor (RSC Sensor).
 *
 * The RSC Collector is multi-instantiated, one instance is created for each connection for
 * which the profile will be enabled and each of these instances will hava a different task ID.
 * Thus, it is very important for the application to keep the source task ID of the first received
 * RSCPC_CMP_EVT message to be able to communicate with the peer device linked to this task ID 
 * once it has been enabled.
 *
 *****************************************************************************************
 */
/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_RSCPC_API Running Speed and Cadence Profile Client API
 * @ingroup APP_RSCPC
 * @brief Running Speed and Cadence Profile Client API
 *
 * Running Speed and Cadence Profile Client APIs are used by APP to enable/disable 
 * Running Speed and Cadence Profile Client Role, to read/write speed-related and cadence-related 
 * data from a Running Speed and Cadence sensor. The RSCPC task for this role will discover 
 * the RSCPS present on the peer server, after establishing connection, and will allow configuration 
 * of the RSCPS attributes if so required. This file contain the implementation of this API. 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_RSC_COLLECTOR
#include "rscpc.h"
#include "rscpc_task.h"
#include "app_rscpc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Running Speed and Cadence profile Collector - at connection.
 *
 ****************************************************************************************
 */
void app_rscpc_enable_req(struct rscpc_rscs_content *rscs, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Read the value of an attribute in the peer device database.
 *
 ****************************************************************************************
 */
void app_rscpc_read_req(uint8_t read_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Configure sending of notification/indication in the peer device database.
 *
 ****************************************************************************************
 */
void app_rscpc_cfg_ntfind_req(uint8_t desc_code, uint16_t ntfind_cfg, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Send the value of the SC Control Point.
 *
 ****************************************************************************************
 */
void app_rscpc_ctnl_pt_cfg_req(struct rscp_sc_ctnl_pt_req *sc_ctnl_pt, uint16_t conhdl);

#endif /* BLE_RSC_COLLECTOR */

/// @} APP_RSCPC_API

#endif // APP_RSCPC_H_
