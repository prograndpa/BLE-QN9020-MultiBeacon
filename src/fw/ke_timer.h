/**
 ****************************************************************************************
 *
 * @file ke_timer.h
 *
 * @brief This file contains the definitions used for timer management
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 24203 $
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *
 * @file ke_timer.h
 *
 * @brief NXP revised and cut.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _KE_TIMER_H_
#define _KE_TIMER_H_

/**
 ****************************************************************************************
 * @defgroup TIMER BT Time
 * @ingroup KERNEL
 * @brief Timer management module.
 *
 * This module implements the functions used for managing kernel timers.
 *
 ****************************************************************************************
 */

#include "ke_msg.h"              // kernel message defines

/*
 * DEFINITIONS
 ****************************************************************************************
 */
/// Timers can be set up to 300s in the future
#define KE_TIMER_DELAY_MAX      30000

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Timer Object
struct ke_timer
{
    /// next ke timer
    struct ke_timer *next;
    /// message identifier
    ke_msg_id_t     id;
    /// task identifier
    ke_task_id_t    task;
    /// time value
    uint16_t        time;
};

/*
 * FUNCTION PROTOTYPES
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Set a timer.
 *
 * The function first cancel the timer if it is already existing, then
 * it creates a new one. The timer can be one-shot or periodic, i.e. it
 * will be automatically set again after each trigger.
 *
 * When the timer expires, a message is sent to the task provided as
 * argument, with the timer id as message id.
 *
 * The timer is programmed in time units (TU is 10ms).
 *
 * @param[in] timer_id      Timer identifier (message identifier type).
 * @param[in] task_id       Task identifier which will be notified
 * @param[in] delay         Delay in time units.
 ****************************************************************************************
 */
typedef void (*p_ke_timer_set)(ke_msg_id_t const timer_id, ke_task_id_t const task, uint16_t const delay);
#define ke_timer_set ((p_ke_timer_set)(_ke_timer_set))

/**
 ****************************************************************************************
 * @brief Remove an registered timer.
 *
 * This function search for the timer identified by its id and its task id.
 * If found it is stopped and freed, otherwise an error message is returned.
 *
 * @param[in] timer_id  Timer identifier.
 * @param[in] task      Task identifier.
 ****************************************************************************************
 */
typedef void (*p_ke_timer_clear)(ke_msg_id_t const timerid, ke_task_id_t const task);
#define ke_timer_clear ((p_ke_timer_clear)(_ke_timer_clear))

/**
 ****************************************************************************************
 * @brief Set a timer using accurate time.
 *
 * When the timer expires, a message is sent to the task provided as
 * argument, with the timer id as message id.
 *
 * The timer is programmed in BLE absolute time
 *
 * @param[in] timer_id      Timer identifier (message identifier type).
 * @param[in] task_id       Task identifier which will be notified
 * @param[in] clk_10ms      Time count by 10ms.
 ****************************************************************************************
 */
typedef void (*p_ke_accurate_timer_set)(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t const clk_10ms);
#define ke_accurate_timer_set ((p_ke_accurate_timer_set)(_ke_accurate_timer_set))

/// @} TIMER

#endif // _KE_TIMER_H_
