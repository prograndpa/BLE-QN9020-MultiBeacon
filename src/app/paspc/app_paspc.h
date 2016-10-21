/**
 ****************************************************************************************
 *
 * @file app_paspc.h
 *
 * @brief Application PASPC API
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

#ifndef APP_PASPC_H_
#define APP_PASPC_H_

/**
 ****************************************************************************************
 * @addtogroup APP_PASPC_API Phone Alert Status Profile client.
 * @ingroup APP_PASPC
 * @brief Phone Alert Status Profile Server
 *
 * The Phone Alert Status profile is used to obtain the Phone Alert Status exposed by the
 * Phone Alert Status service in the peer device. The information of Alert Status and
 * Ringer Setting of a phone can be received and changed by the Phone Alert Status
 * service. This profile also enables the device to configure ringer status on the peer
 * device. 
 * 
 * @{
 ****************************************************************************************
 */

/// @} APP_PASPC_API

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_PAS_CLIENT
#include "paspc.h"
#include "paspc_task.h"
#include "app_paspc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Phone Alert Status Profile Client - at connection.
 ****************************************************************************************
 */
void app_paspc_enable_req(struct paspc_pass_content *pass, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Read the value of an attribute in the peer device database.
 ****************************************************************************************
 */
void app_paspc_read_req(uint8_t read_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Write the value of an attribute in the peer device database.
 ****************************************************************************************
 */
void app_paspc_wr_char_req(uint8_t write_code, union write_value_tag *value, uint16_t conhdl);
#endif // BLE_PAS_CLIENT

/// @} APP_PASPC_API

#endif // APP_PASPC_H_
