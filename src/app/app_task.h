/**
 ****************************************************************************************
 *
 * @file app_task.h
 *
 * @brief APP Task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_TASK_H_
#define _APP_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APPTASK Task
 * @ingroup APP
 * @brief Routes ALL messages to/from APP block.
 *
 * The APPTASK is the block responsible for bridging the final application with the
 * QNBLE software host stack. It communicates with the different modules of the BLE host,
 * i.e. @ref SMP, @ref GAP and @ref GATT.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ke_task.h"
#include "ke_msg.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// number of APP Instance
#define APP_IDX_MAX                                 0x01

/// states of APP task
enum
{
    /// Init state
    APP_INIT = 1,
    /// Idle state
    APP_IDLE,
    /// Advertising state
    APP_ADV,
    /// Scan state
    APP_SCAN,
    /// Number of defined states.
    APP_STATE_MAX
};

/// App Message Interface
enum
{
    /// Debug UART received data
    APP_SYS_UART_DATA_IND = KE_FIRST_MSG(TASK_APP),
    APP_SYS_LED_1_TIMER,
    APP_SYS_LED_2_TIMER,
    APP_ADV_INTV_UPDATE_TIMER,
    APP_SYS_RCO_CAL_TIMER,
    APP_SYS_32K_XTAL_WAKEUP_TIMER,

    APP_SYS_TIME_CHECK_TIMER,

    APP_SYS_BUTTON_1_TIMER,
    APP_SYS_BUTTON_2_TIMER,
    // for ancsc
    ANCS_ENABLE_SUVPER_TIMER0,
    ANCS_ENABLE_SUVPER_TIMER1,
    // end
    APP_HTPT_PERIOD_MEAS_TIMER,
    APP_HTPT_IDLE_CONNECTION_TIMEOUT_TIMER,
    
    APP_PROXR_ALERT_STOP_TIMER,
    APP_BLPS_PRESSURE_TIMER,
    APP_HRPS_TIMER,
    APP_TIPS_CURRENT_TIME_TIMER,
    APP_SCPPS_SCAN_REFRESH_TIMER,
    APP_BASS_BATT_LEVEL_TIMER,
    APP_GLPS_MEAS_SEND_TIMER,
    APP_RSCPS_MEAS_SEND_TIMER,
    APP_CSCPS_MEAS_SEND_TIMER,
    APP_PASPS_UPDATE_ALERT_STATUS_TIMER,
    APP_PASPS_UPDATE_RINGER_TIMER,

    APP_HOGPD_BOOT_KB_IN_REPORT_TIMER,
    APP_HOGPD_BOOT_MOUSE_IN_REPORT_TIMER,
    APP_HOGPD_REPORT_TIMER,
		APP_BEACON_CHG_CTX_TIMER,
    APP_MSG_MAX
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler app_state_handler[APP_STATE_MAX];

extern const struct ke_state_handler app_default_handler;

extern ke_state_t app_state[APP_IDX_MAX];

extern void task_app_desc_register(void);

#endif // APP_TASK_H_
