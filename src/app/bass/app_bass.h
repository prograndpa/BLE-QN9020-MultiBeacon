/**
 ****************************************************************************************
 *
 * @file app_bass.h
 *
 * @brief Application BASS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_BASS_H_
#define APP_BASS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_BASS_API Battery 'Profile' Server
 * @ingroup APP_BASS
 * @brief Battery 'Profile' Server
 *
 * The Battery Service exposes the Battery Level of a single battery or set of batteries in a device.
 * The Battery Level may either be read, or be enabled for notification by peer device.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_BATT_SERVER
#include "bass.h"
#include "bass_task.h"
#include "app_bass_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the battery service database - at initiation
 * 
 ****************************************************************************************
 */
void app_bass_create_db(uint8_t bas_nb, uint8_t *features);

/*
 ****************************************************************************************
 * @brief Start the battery service server - at connection
 *
 ****************************************************************************************
 */
void app_bass_enable_req(uint16_t conhdl, uint8_t bas_nb, uint8_t sec_lvl, uint8_t con_type, 
                         uint16_t *batt_level_ntf_cfg,  uint8_t *old_batt_lvl,
                         uint8_t *current_batt_lvl, struct prf_char_pres_fmt *batt_level_pres_format);

/*
 ****************************************************************************************
 * @brief Send the battery level update - at connection
 *
 ****************************************************************************************
 */
void app_bass_batt_level_upd_req(uint16_t conhdl, uint8_t bas_instance, uint8_t batt_level);

#endif // BLE_BATT_SERVER

/// @} APP_BASS_API

#endif // APP_BASS_H_
