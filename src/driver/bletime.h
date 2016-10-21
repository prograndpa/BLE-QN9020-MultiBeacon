/**
 ****************************************************************************************
 *
 * @file bletime.c
 *
 * @brief QN9020 time support.
 * 
 * @note
 * 1. Deep sleep will lead to hardware counter stop, so call qn_time_set() 
 *    function to reset it or forbid deep sleep after leave deep sleep.
 * 2. Please include this file in your "usr_design.h"
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */



#ifndef __BLE_TIME_H__
#define __BLE_TIME_H__

#define QN_BLE_TIME_SUPPORT

#include <time.h>
#include <stdint.h>
#include "ke_msg.h"
#include "ke_task.h"
#include "lib.h"

// time struct
typedef struct
{
    uint16_t year;     // 1970+
    uint8_t  month;    // 1-12
    uint8_t  day;      // 1-31
    uint8_t  hour;     // 0-23
    uint8_t  minutes;  // 0-59
    uint8_t  seconds;  // 0-59
} qn_tm_t;

/**
 ****************************************************************************************
 * @brief ble time initialize function.
 *
 * @description 
 * This function must be called after receive GAP_SET_MODE_REQ_CMP_EVT message, as the same 
 * time the state of "TASK_APP" is "APP_INIT".
 ****************************************************************************************
 */
extern void qn_time_init(void);

/**
 ****************************************************************************************
 * @brief set time with year,month,day,hour,minute,second
 ****************************************************************************************
 */
extern void qn_time_get(qn_tm_t *ptm);

/**
 ****************************************************************************************
 * @brief set time with year,month,day,hour,minute,second
 ****************************************************************************************
 */
extern bool qn_time_set(const qn_tm_t *ptm);


/**
 ****************************************************************************************
 * @brief ble time check timer
 ****************************************************************************************
 */
extern int app_sys_time_check_timer_handler(ke_msg_id_t const msgid, void const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

#endif



