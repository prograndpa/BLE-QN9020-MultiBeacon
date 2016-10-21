/**
 ****************************************************************************************
 *
 * @file usr_design.c
 *
 * @brief Product related design.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  USR
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "app_env.h"
#include "led.h"
#include "uart.h"
#include "lib.h"
#include "usr_design.h"
#include "gpio.h"
#include "button.h"
#include "sleep.h"

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define LED_ON_DUR_IDLE                 0
#define LED_OFF_DUR_IDLE                0xffff
#define EVENT_BUTTON1_PRESS_ID          0
#define EVENT_BUTTON2_PRESS_ID          1
#define EVENT_BEACON_CHG_CTX_TIMER_ID		2
#define GAP_ADV_INTV1                   0x00aa
#define GAP_ADV_INTV2                   0x0100

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct app_env_tag app_env;
struct usr_env_tag usr_env = {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE};
uint8_t adv1_data[] = {0x02,GAP_AD_TYPE_FLAGS,GAP_BR_EDR_NOT_SUPPORTED,0x04,GAP_AD_TYPE_SHORTENED_NAME,
                        'N', 'X' , 'P'}; // "NXP"
uint8_t adv2_data[] = {0x02,GAP_AD_TYPE_FLAGS,GAP_BR_EDR_NOT_SUPPORTED,0x04,GAP_AD_TYPE_SHORTENED_NAME,
                        'P', 'X' , 'N'}; // "NXP"
uint8_t beacon_data[3][30] ={{0x02,GAP_AD_TYPE_FLAGS,GAP_BR_EDR_NOT_SUPPORTED,0x1A,//length
											GAP_AD_TYPE_MANU_SPECIFIC_DATA,0x4C,0x00,0x02,0x15,// iBeacon Prefix(9 bytes)
                      0x01,0x12,0x23,0x34,0x45,0x56,0x67,0x78,0x89,0x9a,0xab,0xbc,0xcd,0xde,0xef,0xf0,// 16 bytes UUID
                      0x01,0x02, // Major
                      0x01,0x01, // Minor
                      0xc3},
											{0x02,GAP_AD_TYPE_FLAGS,GAP_BR_EDR_NOT_SUPPORTED,0x1A,//length
											GAP_AD_TYPE_MANU_SPECIFIC_DATA,0x4C,0x00,0x02,0x15,// iBeacon Prefix(9 bytes)
                      0x01,0x12,0x23,0x34,0x45,0x56,0x67,0x78,0x89,0x9a,0xab,0xbc,0xcd,0xde,0xef,0xf0,// 16 bytes UUID
                      0x01,0x02, // Major
                      0x02,0x02, // Minor
                      0xc3},
											{0x02,GAP_AD_TYPE_FLAGS,GAP_BR_EDR_NOT_SUPPORTED,0x1A,//length
											GAP_AD_TYPE_MANU_SPECIFIC_DATA,0x4C,0x00,0x02,0x15,// iBeacon Prefix(9 bytes)
                      0x01,0x12,0x23,0x34,0x45,0x56,0x67,0x78,0x89,0x9a,0xab,0xbc,0xcd,0xde,0xef,0xf0,// 16 bytes UUID
                      0x01,0x02, // Major
                      0x03,0x03, // Minor
                      0xc3}
											}; // "NXP"
uint8_t scan_data[] = {0x05,0x12,0x06,0x00,0x80,0x0c}; //Slave Connection Interval Range

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Led 1 flash process
 ****************************************************************************************
 */
static void usr_led1_process(void)
{
    if (led_get(1) == LED_ON)
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_off_dur);
    }
    else
    {
        led_set(1, LED_ON);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_on_dur);
    }
}
static void usr_beacon_chg_ctx_process(void)
{
		static uint8_t beacon_ctx_idx = 0;
		ke_evt_clear(1UL << EVENT_BEACON_CHG_CTX_TIMER_ID);
    if (APP_ADV == ke_state_get(TASK_APP))
    {
                    // stop adv
                    app_gap_adv_stop_req();

                    ke_state_set(TASK_APP, APP_IDLE);
#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
     }
		if (APP_IDLE == ke_state_get(TASK_APP))
		{
				// start adv
				app_gap_adv_start_req(GAP_GEN_DISCOVERABLE,
															beacon_data[beacon_ctx_idx], sizeof(beacon_data[beacon_ctx_idx]), 
															scan_data, sizeof(scan_data),
															GAP_ADV_INTV1, GAP_ADV_INTV2);
				if(beacon_ctx_idx++ > 2)
					beacon_ctx_idx = 0;
				ke_state_set(TASK_APP, APP_ADV);
				ke_timer_set(APP_BEACON_CHG_CTX_TIMER, TASK_APP, 10);
#if (QN_DEEP_SLEEP_EN)
						// prevent entering into deep sleep mode
						sleep_set_pm(PM_SLEEP);
#endif
     }

	}
/**
 ****************************************************************************************
 * @brief   Application task message handler
 ****************************************************************************************
 */
void app_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
}

/**
 ****************************************************************************************
 * @brief Handles LED status timer.
 *
 * @param[in] msgid      APP_SYS_UART_DATA_IND
 * @param[in] param      Pointer to struct app_uart_data_ind
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_led_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (msgid == APP_SYS_LED_1_TIMER)
    {
        usr_led1_process();
    }

    return (KE_MSG_CONSUMED);
}
int app_beacon_chg_ctx_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (msgid == APP_BEACON_CHG_CTX_TIMER)
    {
        ke_evt_set(1UL << EVENT_BEACON_CHG_CTX_TIMER_ID);
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief   Restore peripheral setting after wakeup
 ****************************************************************************************
 */
void usr_sleep_restore(void)
{
#if QN_DBG_PRINT
    uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_9600);
    uart_tx_enable(QN_DEBUG_UART, MASK_ENABLE);
    uart_rx_enable(QN_DEBUG_UART, MASK_ENABLE);
#endif
}

/**
 ****************************************************************************************
 * @brief Handles button press after cancel the jitter.
 *
 * @param[in] msgid     Id of the message received
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_button_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_SYS_BUTTON_1_TIMER:
            // make sure the button is pressed
            if (gpio_read_pin(BUTTON1_PIN) == GPIO_LOW)
            {
                if (APP_IDLE == ke_state_get(TASK_APP))
                {
                    // start adv
                    app_gap_adv_start_req(GAP_GEN_DISCOVERABLE,
                                          adv1_data, sizeof(adv1_data), 
                                          scan_data, sizeof(scan_data),
                                          GAP_ADV_INTV1, GAP_ADV_INTV2);

                    ke_state_set(TASK_APP, APP_ADV);
										ke_timer_set(APP_BEACON_CHG_CTX_TIMER, TASK_APP, 10);
#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
                }
                else if (APP_ADV == ke_state_get(TASK_APP))
                {
                    // stop adv
                    app_gap_adv_stop_req();

                    ke_state_set(TASK_APP, APP_IDLE);
#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
                }
            }
            break;
				case APP_SYS_BUTTON_2_TIMER:
            // make sure the button is pressed
            if (gpio_read_pin(BUTTON2_PIN) == GPIO_LOW)
            {
                if (APP_IDLE == ke_state_get(TASK_APP))
                {
                    // start adv
                    app_gap_adv_start_req(GAP_GEN_DISCOVERABLE,
                                          adv2_data, sizeof(adv2_data), 
                                          scan_data, sizeof(scan_data),
                                          GAP_ADV_INTV1, GAP_ADV_INTV2);

                    ke_state_set(TASK_APP, APP_ADV);
#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
                }
                else if (APP_ADV == ke_state_get(TASK_APP))
                {
                    // stop adv
                    app_gap_adv_stop_req();

                    ke_state_set(TASK_APP, APP_IDLE);
#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
                }
            }
            break;

        default:
            ASSERT_ERR(0);
            break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_button1_press_handler(void)
{
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep) 
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif

    // delay 20ms to debounce
    ke_timer_set(APP_SYS_BUTTON_1_TIMER, TASK_APP, 2);
    ke_evt_clear(1UL << EVENT_BUTTON1_PRESS_ID);
}
void app_event_button2_press_handler(void)
{
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep) 
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif

    // delay 20ms to debounce
    ke_timer_set(APP_SYS_BUTTON_2_TIMER, TASK_APP, 2);
    ke_evt_clear(1UL << EVENT_BUTTON2_PRESS_ID);
}

/**
 ****************************************************************************************
 * @brief   Button 1 click callback
 * @description
 *  Button 1 is used to enter adv mode.
 ****************************************************************************************
 */
void usr_button1_cb(void)
{
    // If BLE is in the sleep mode, wakeup it.
    if (ble_ext_wakeup_allow())
    {
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
        if (sleep_env.deep_sleep)
        {
            wakeup_32k_xtal_switch_clk();
        }
#endif

        sw_wakeup_ble_hw();
    }

    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_BUTTON1_PRESS_ID);
}
void usr_button2_cb(void)
{
    // If BLE is in the sleep mode, wakeup it.
    if (ble_ext_wakeup_allow())
    {
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
        if (sleep_env.deep_sleep)
        {
            wakeup_32k_xtal_switch_clk();
        }
#endif

        sw_wakeup_ble_hw();
    }

    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_BUTTON2_PRESS_ID);
}
/**
 ****************************************************************************************
 * @brief   All GPIO interrupt callback
 ****************************************************************************************
 */
void gpio_interrupt_callback(enum gpio_pin pin)
{
    switch(pin)
    {
        case BUTTON1_PIN:
            //Button 1 is used to enter adv mode.
            usr_button1_cb();
            break;
				case BUTTON2_PIN:
            //Button 2 is used to enter adv mode.
            usr_button2_cb();
            break;
#if (defined(QN_TEST_CTRL_PIN))
        case QN_TEST_CTRL_PIN:
            //When test controll pin is changed to low level, this function will reboot system.
            gpio_disable_interrupt(QN_TEST_CTRL_PIN);
            syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_REBOOT_SYS);
            break;
#endif

        default:
            break;
    }
}


/**
 ****************************************************************************************
 * @brief   User initialize
 ****************************************************************************************
 */
void usr_init(void)
{
    if (KE_EVENT_OK != ke_evt_callback_set(EVENT_BUTTON1_PRESS_ID, 
                                            app_event_button1_press_handler))
    {
        ASSERT_ERR(0);
    }
		if (KE_EVENT_OK != ke_evt_callback_set(EVENT_BUTTON2_PRESS_ID, 
                                            app_event_button2_press_handler))
    {
        ASSERT_ERR(0);
    }
		if (KE_EVENT_OK != ke_evt_callback_set(EVENT_BEACON_CHG_CTX_TIMER_ID, 
                                            usr_beacon_chg_ctx_process))
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief Perform Profile initialization
 ****************************************************************************************
 */
void prf_init(void)
{
}

/**
 ****************************************************************************************
 * @brief dispatch disconnection complete event to profiles.
 *
 * @param[in] status        disconnection complete event status
 * @param[in] reason        detach reason
 * @param[in] conhdl        connection handle
 *
 ****************************************************************************************
 */
void prf_dispatch_disconnect(uint8_t status, uint8_t reason, uint16_t conhdl, uint8_t idx)
{
}

/**
 ****************************************************************************************
 * @brief Initialize the application task environment.
 *
 ****************************************************************************************
 */
void app_init(void)
{
    // Register APP task into kernel
    task_app_desc_register();
    ke_state_set(TASK_APP, APP_INIT);
}
/// @} USR

