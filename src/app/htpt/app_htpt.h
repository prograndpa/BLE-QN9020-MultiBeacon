/**
 ****************************************************************************************
 *
 * @file app_htpt.h
 *
 * @brief Application HTPT API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_HTPT_H_
#define APP_HTPT_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HTPT_API Health Thermometer Profile Thermometer
 * @ingroup APP_HTPT
 * @brief Health Thermometer Profile Thermometer
 *
 * An actual thermometer device does not exist on current platform, so measurement values
 * that would come from a driver are replaced by simple counters sent at certain intervals
 * following by the profile attributes configuration.
 * When a measurement interval has been set to a non-zero value in a configuration connection,
 * once reconnected, TH will send regular measurement INDs if Temp Meas Char Cfg is set to
 * indicate and using the Meas Intv Value. The INDs will continue until meas interval is set to 0
 * or connection gets disconnected by C. Measurements should be stored even so, until
 * profile is disabled.
 *
 * If the measurement interval has been set to 0, then if Intermediate Temp is set to be
 * notified and Temp Meas to be indicated, then a timer of fixed length simulates
 * sending several NTF before and indication of a "stable" value. This fake behavior
 * should be replaced once a real driver exists. If Intermediary Temp cannot be notified,
 * just send the indication, if neither can be sent (the configuration connection should
 * never leave this like this) then disconnect.
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

#if BLE_HT_THERMOM
#include "htpt.h"
#include "htpt_task.h"
#include "app_htpt_task.h"

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
 * @brief Create the health thermometer server service database - at initiation    
 *
 ****************************************************************************************
 */
void app_htpt_create_db(uint16_t valid_range_min, uint16_t valid_range_max, uint8_t features);

/*
 ****************************************************************************************
 * @brief Start the health thermometer server service - at connection      
 *
 ****************************************************************************************
 */
void app_htpt_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t temp_meas_ind_en,
                         uint16_t interm_temp_ntf_en, uint16_t meas_intv_ind_en, uint16_t meas_intv);

/*
 ****************************************************************************************
 * @brief Send Temperature Measurement value    
 *
 ****************************************************************************************
 */
void app_htpt_temp_send(uint16_t conhdl, struct htp_temp_meas *temp_meas, uint8_t flag_stable_meas);

/*
 ****************************************************************************************
 * @brief Send Measurement Interval value    
 *
 ****************************************************************************************
 */
void app_htpt_measurement_intv_send(uint16_t conhdl, uint16_t meas_intv);

/*
 ****************************************************************************************
 * @brief Send temperature type     
 *
 ****************************************************************************************
 */
void app_htpt_temp_type_send(uint8_t value);

#endif // BLE_HT_THERMOM

/// @} APP_HTPT_API

#endif // APP_HTPT_H_
