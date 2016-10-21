/**
 ****************************************************************************************
 *
 * @file bletime.c
 *
 * @brief QN9020 time support.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */


#include "bletime.h"

// the value range is (0-30000)
#define SYS_TIME_CHECK_PERIOD   (29999)

#define BASE_TIME_CNT_TO_S(n)     (((n) + 50 ) / 100)  //round-off
#define MAX_TICK_COUNT_10MS       (0x7FFFFF)

static volatile uint32_t s_pre_tick_count_10ms = 0;
static volatile uint64_t s_tick_count_10ms = 0;

/**
 ****************************************************************************************
 * @brief Set time with seconds.
 ****************************************************************************************
 */
void set_time_sec(time_t new_sec)
{
    s_tick_count_10ms = (uint64_t)new_sec * 100;
    s_pre_tick_count_10ms = ke_time();
}

/**
 ****************************************************************************************
 * @brief get time with seconds.
 ****************************************************************************************
 */
time_t get_time_sec(void)
{
    const uint32_t cur_tick_count_10ms = ke_time();
    uint32_t diff = 0;

    if(cur_tick_count_10ms >= s_pre_tick_count_10ms)
    {
        diff = cur_tick_count_10ms - s_pre_tick_count_10ms;
    }
    else // over flow
    {
        diff = MAX_TICK_COUNT_10MS - s_pre_tick_count_10ms + cur_tick_count_10ms;
    }

    return BASE_TIME_CNT_TO_S(s_tick_count_10ms + diff);
}

/**
 ****************************************************************************************
 * @brief trigger time check
 ****************************************************************************************
 */
void time_check_trigger(void)
{
    const uint32_t cur_tick_count_10ms = ke_time();
    uint32_t diff = 0;

    if(cur_tick_count_10ms >= s_pre_tick_count_10ms)
    {
        diff = cur_tick_count_10ms - s_pre_tick_count_10ms;
    }
    else // over flow
    {
        diff = MAX_TICK_COUNT_10MS - s_pre_tick_count_10ms + cur_tick_count_10ms;
    }
    
    s_tick_count_10ms += diff;
    s_pre_tick_count_10ms = cur_tick_count_10ms;

    ke_timer_set(APP_SYS_TIME_CHECK_TIMER, TASK_APP, SYS_TIME_CHECK_PERIOD);
}

/**
 ****************************************************************************************
 * @brief set time with year,month,day,hour,minute,second
 ****************************************************************************************
 */
bool qn_time_set(const qn_tm_t *ptm)
{
    struct tm ltime = {0};

    if(   ptm->day < 1
       || ptm->day > 31
       || ptm->hour > 23
       || ptm->minutes > 59
       || ptm->month < 1
       || ptm->month > 12
       || ptm->seconds > 59
       || ptm->year < 1970)
        return false;

    ltime.tm_sec  = ptm->seconds;
    ltime.tm_min  = ptm->minutes;
    ltime.tm_hour = ptm->hour;
    ltime.tm_mday = ptm->day;
    ltime.tm_mon  = ptm->month - 1;
    ltime.tm_year = ptm->year - 1900;

    set_time_sec(mktime(&ltime));

    return true;
}

/**
 ****************************************************************************************
 * @brief get time with year,month,day,hour,minute,second
 ****************************************************************************************
 */
void qn_time_get(qn_tm_t *ptm)
{
    struct tm *ltime;
    const time_t sec = get_time_sec();

    // function inside memery manage, and should use gmtime() function but there have a problem
    ltime = localtime(&sec);

    ptm->seconds = ltime->tm_sec;
    ptm->minutes = ltime->tm_min;
    ptm->hour    = ltime->tm_hour;
    ptm->day     = ltime->tm_mday;
    ptm->month   = ltime->tm_mon + 1;
    ptm->year    = ltime->tm_year + 1900;
}

/**
 ****************************************************************************************
 * @brief ble time initialize function.
 *
 * @description 
 * This function must be called after receive GAP_SET_MODE_REQ_CMP_EVT message, as the same 
 * time the state of "TASK_APP" is "APP_INIT".
 ****************************************************************************************
 */
void qn_time_init(void)
{
    time_check_trigger();
}

/**
 ****************************************************************************************
 * @brief ble time check timer
 ****************************************************************************************
 */
int app_sys_time_check_timer_handler(ke_msg_id_t const msgid, void const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    time_check_trigger();

    return (KE_MSG_CONSUMED);
}


