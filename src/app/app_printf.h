/**
 ****************************************************************************************
 *
 * @file app_printf.c
 *
 * @brief Application PRINTF API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _APP_PRINTF_H_
#define _APP_PRINTF_H_
#include <stdarg.h>
#include "app_config.h"
#if QN_STD_PRINTF
#include <stdio.h>
#endif

#if QN_DBG_PRINT

#define QSPRINTF qsprintf

#if QN_STD_PRINTF
    #define QPRINTF printf
#else
    #define QPRINTF qprintf
#endif

#define QTRACE qtrace

#else

#define QSPRINTF(buf, fmt, args)
#define QPRINTF(fmt, ...)
#define QTRACE(data, len, dir, fmt)

#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if QN_DBG_PRINT

/*
 ****************************************************************************************
 * @brief QN sprintf function
 ****************************************************************************************
 */
extern int qsprintf(char *buf, const char *fmt, va_list args);

#if !QN_STD_PRINTF
/*
 ****************************************************************************************
 * @brief QN printf function
 ****************************************************************************************
 */
extern int qprintf(const char *fmt, ... );
#endif

/*
 ****************************************************************************************
 * @brief Trace data with length and format
 ****************************************************************************************
 */
extern void qtrace(uint8_t *data, uint16_t len, bool dir, uint8_t fmt);

#endif

#endif

