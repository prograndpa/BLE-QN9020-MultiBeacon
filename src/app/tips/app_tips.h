/**
 ****************************************************************************************
 *
 * @file app_tips.h
 *
 * @brief Application TIPS API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_TIPS_H_
#define APP_TIPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP_TIPS_API Time Profile Server
 * @ingroup APP_TIPS
 * @brief Time Profile Server
 *
 * The Bluetooth Low Energy Time profile enables the user to obtain the current date and time,
 * and related information such as time zone as exposed by the Current Time service in the peer
 * device. Information of when next change of daylight savings time (DST) will occur can be
 * retrieved from the peer exposed by the Next DST Change service. This profile also enables a
 * device to request updating the time on the peer device as exposed by the Reference Time
 * Update Service.
 * Within the profile, two roles can be supported: Client and Server.
 * The Time Server shall be a server. The Time Client shall be a client.
 * The role of server is meant to be activated on the device that acts as Time Server and sends time
 * values to the Client.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_TIP_SERVER
#include "tip_common.h"
#include "tips.h"
#include "tips_task.h"
#include "app_tips_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the time server database - at initiation   
 *
 ****************************************************************************************
 */
void app_tips_create_db(uint8_t features);

/*
 ****************************************************************************************
 * @brief Start the time server role - at connection  
 * 
 ****************************************************************************************
 */
void app_tips_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t current_time_ntf_en);

/*
 ****************************************************************************************
 * @brief Send update current time request - at connection  
 *
 ****************************************************************************************
 */
void app_tips_upd_curr_time_req(uint16_t conhdl, struct tip_curr_time *current_time, uint8_t enable_ntf_send);

/*
 ****************************************************************************************
 * @brief Send update local time info request - at connection 
 *
 ****************************************************************************************
 */
void app_tips_upd_local_time_info_req(uint16_t conhdl, struct tip_loc_time_info *loc_time_info);

/*
 ****************************************************************************************
 * @brief Send update reference time info request - at connection 
 *
 ****************************************************************************************
 */
void app_tips_upd_ref_time_info_req(uint16_t conhdl, struct tip_ref_time_info *ref_time_info);

/*
 ****************************************************************************************
 * @brief Send update time DST request - at connection   
 *
 ****************************************************************************************
 */
void app_tips_upd_time_dst_req(uint16_t conhdl, struct tip_time_with_dst *time_with_dst);

/*
 ****************************************************************************************
 * @brief Send update time update state request - at connection  
 *
 ****************************************************************************************
 */
void app_tips_upd_time_upd_state_req(uint16_t conhdl, struct tip_time_upd_state *time_upd_state);

#endif // BLE_TIP_SERVER

/// @} APP_TIPS_API

#endif // APP_TIPS_H_
