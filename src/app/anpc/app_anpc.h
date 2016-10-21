/**
 ****************************************************************************************
 *
 * @file app_anpc.h
 *
 * @brief Application ANPC API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_ANPC_H_
#define APP_ANPC_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

#if BLE_AN_CLIENT
#include "anpc.h"
#include "anpc_task.h"
#include "app_anpc_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Alert Notification Client Role - at connection
 *
 ****************************************************************************************
 */
void app_anpc_enable_req(struct anp_cat_id_bit_mask *new_alert_enable,
                         struct anp_cat_id_bit_mask *unread_alert_enable,
                         struct anpc_ans_content *ans,
                         uint16_t conhdl);
/*
 ****************************************************************************************
 * @brief Generic message to read a New Alert CFG or Unread Alert Status CFG characteristic
 *
 ****************************************************************************************
 */
void app_anpc_rd_char_req(uint8_t read_code, uint16_t conhdl);

/*
 ****************************************************************************************
 * @brief Generic message to write characteristic
 *
 ****************************************************************************************
 */
void app_anpc_wr_char_req(uint8_t write_code, union anpc_write_value_tag *value, uint16_t conhdl);

#endif /* BLE_AN_CLIENT */

/// @} APP_ANPC_API

#endif // APP_ANPC_H_
