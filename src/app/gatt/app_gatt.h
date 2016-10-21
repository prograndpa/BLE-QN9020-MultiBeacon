/**
 ****************************************************************************************
 *
 * @file app_gatt.h
 *
 * @brief Application GATT API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_GATT_H_
#define _APP_GATT_H_

/**
 ****************************************************************************************
 * @addtogroup APP_GATT_API Generic Attribute Profile API
 * @ingroup APP_GATT
 * @brief Generic Attribute Profile API
 *
 *  The Generic Attribute Profile (GATT) defines the service framework using the Attribute 
 *  Protocol for discovering services and for reading and writing characteristic values on
 *  a peer device. 
 *
 *  GATT APIs are used by APP to serch the service details which include UUID, start handle
 *  and end handle from the peer device.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gatt.h"
#include "gatt_task.h"
#include "attm.h"
#include "attc_task.h"
#include "app_gatt_task.h"

/*
 ****************************************************************************************
 * @brief Gatt Discovery Service request
 *
 ****************************************************************************************
 */
void app_gatt_disc_svc_req(uint8_t req_type, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Gatt Characteristic discovery request
 *
 ****************************************************************************************
 */
void app_gatt_disc_char_req(uint8_t req_type, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Gatt Characteristic descriptor discovery request
 *
 ****************************************************************************************
 */
void app_gatt_disc_char_desc_req(uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Gatt Read Characteristic request
 *
 ****************************************************************************************
 */
void app_gatt_read_char_req(uint8_t req_type, uint16_t conhdl, uint16_t valhdl);

/*
 ****************************************************************************************
 * @brief Gatt Write Characteristic request
 *
 ****************************************************************************************
 */
void app_gatt_write_char_req(uint8_t req_type, uint16_t conhdl, uint16_t valhdl, uint16_t val_len, uint8_t *pdata);

/*
 ****************************************************************************************
 * @brief Gatt Write reliable characteristic request
 *
 ****************************************************************************************
 */
void app_gatt_write_reliable_req(uint16_t conhdl, uint8_t nb_writes, uint8_t auto_execute, struct gatt_reliable_write *data_write);

/*
 ****************************************************************************************
 * @brief Gatt Write reliable characteristic request
 *
 ****************************************************************************************
 */
void app_gatt_execute_write_char_req(uint16_t conhdl, uint8_t exe_wr_ena);

/*
 ****************************************************************************************
 * @brief Gatt Send notification request
 *
 ****************************************************************************************
 */
void app_gatt_notify_req(uint16_t conhdl, uint16_t charhdl);
/*
 ****************************************************************************************
 * @brief Gatt Send read request confirmation. *//**
 ****************************************************************************************
 */
void app_gatt_read_req_cfm(uint16_t conhdl, uint16_t atthdl, uint16_t val_len, uint8_t *valule);
/// @} APP_GATT_API

#endif // _APP_GATT_H_


