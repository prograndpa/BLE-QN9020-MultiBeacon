/**
 ****************************************************************************************
 *
 * @file app_sys.h
 *
 * @brief Application System Functioins
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_SYS_H_
#define _APP_SYS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (QN_DEMO_MENU || QN_EACI)
#if (QN_DEMO_MENU)

#define QN_UART_RX_LEN      0x10

/// Application UART environment context structure
struct app_uart_env_tag
{
    uint8_t len;
    uint8_t buf_rx[QN_UART_RX_LEN];
};
#endif

/// Message of UART data request send to application
struct app_uart_data_req
{
    uint8_t len;
    uint8_t data[1];
};

/// Application handle message of UART received data
struct app_uart_data_ind
{
    uint8_t len;
    uint8_t data[1];
};

#endif

#if QN_DBG_PRINT

/*
 ****************************************************************************************
 * @brief Uart initialization.
 *
 ****************************************************************************************
 */
void app_uart_init(void);

#endif

#endif // _APP_SYS_H_

