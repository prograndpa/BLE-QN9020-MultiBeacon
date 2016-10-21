/**
 ****************************************************************************************
 *
 * @file app_config.h
 *
 * @brief Application Configuration File
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup APP_CONFIG Application Configuration Defines
 * @ingroup APP_CONFIG
 * @brief Application Configuration Defines
 *
 * Application Configuration Defines are used to enable or disable differenct features of
 * the application.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "usr_config.h"

/// Application Version Number
#define QN_APP_VER                  "1.3.9"

/// Software Version
#if (defined(CFG_SW_RELEASE))
    #define QN_SW_RELEASE
#endif

/// QN9020 Chip Version Number
#if (defined(CFG_9020_B2))
    #define QN_9020_B2
#elif (defined(CFG_9020_B4))
    #define QN_9020_B4
#else
    #error "No correct chip version is defined!"
#endif

/// High operating ambient temperature support
#if (defined(CFG_T_VERSION))
    #define QN_T_VERSION
#endif

/// Evaluation Board Indication
#if (defined(CFG_9021_MINIDK))
    #define QN_9021_MINIDK
#elif (defined(CFG_9022_MINIDK_V1_1))
    #define QN_9022_MINIDK_V1_1
#endif

/// External flash
#if (defined(CFG_EXT_FLASH))
    #define QN_EXT_FLASH
#endif

/// No external flash
#if (defined(CFG_NO_FLASH))
    #define QN_NO_FLASH
#endif

/// Work Mode
#define WORK_MODE_SOC               0
#define WORK_MODE_NP                1
#define WORK_MODE_HCI               2

#if (defined(CFG_WM_SOC))
    #define QN_WORK_MODE            WORK_MODE_SOC
#elif (defined(CFG_WM_NP))
    #define QN_WORK_MODE            WORK_MODE_NP
#elif (defined(CFG_WM_HCI))
    #define QN_WORK_MODE            WORK_MODE_HCI
#else
    #define QN_WORK_MODE            WORK_MODE_SOC
#endif

#if (defined(CFG_EACI))
    #define QN_EACI                 1
#else
    #define QN_EACI                 0
#endif

#if (defined(CFG_EAPI))
    #define QN_EAPI                 1
#else
    #define QN_EAPI                 0
#endif

/// Application friendly name setting
#if (defined(CFG_LOCAL_NAME))
    #define QN_LOCAL_NAME           CFG_LOCAL_NAME
#else
    #define QN_LOCAL_NAME           "NXP BLE"
#endif

/// DC-DC enable
#if (defined(CFG_DC_DC))
    #define QN_DC_DC_ENABLE         true
#else
    #define QN_DC_DC_ENABLE         false
    #define CFG_HIGH_PERFORMANCE
#endif

/// 32K RCO
#if (defined(CFG_32K_RCO))
    #define QN_32K_RCO              1
#else
    #define QN_32K_RCO              0
#endif

/// Power mode
#if (defined(CFG_HIGH_PERFORMANCE))
    #define QN_POWER_MODE           HIGH_PERFORMANCE
#else
    #define QN_POWER_MODE           NORMAL_MODE
#endif

/// NVDS write
#if (defined(CFG_NVDS_WRITE))
    #define QN_NVDS_WRITE           1
    #define NVDS_TMP_BUF_SIZE       0x1000
#else
    #define QN_NVDS_WRITE           0
#endif

/// Test controll pin
#if (defined(CFG_TEST_CTRL_PIN))
    #define QN_TEST_CTRL_PIN CFG_TEST_CTRL_PIN
#endif

/// Memory retention
#if (defined(CFG_MEM_RETENTION))
    #define QN_MEM_RETENTION        CFG_MEM_RETENTION
    #define QN_MEM_UNRETENTION      (~(CFG_MEM_RETENTION) & 0xfe)
#else
    #define QN_MEM_RETENTION        (MEM_BLOCK1 | MEM_BLOCK2 | MEM_BLOCK3 | MEM_BLOCK4 | MEM_BLOCK5 | MEM_BLOCK6 | MEM_BLOCK7)
    #define QN_MEM_UNRETENTION      0
#endif

/// Deep sleep
#if (defined(CFG_DEEP_SLEEP))
    #define QN_DEEP_SLEEP           PM_DEEP_SLEEP
    #define QN_DEEP_SLEEP_EN        1
#else
    #define QN_DEEP_SLEEP           PM_SLEEP
    #define QN_DEEP_SLEEP_EN        0
#endif

/// BLE sleep
#if (defined(CFG_BLE_SLEEP))
    #define QN_BLE_SLEEP            true
#else
    #define QN_BLE_SLEEP            false
#endif

/// BLE MAX sleep duration
#if (defined(CFG_BLE_MAX_SLEEP_DUR))
    #define QN_BLE_MAX_SLEEP_DUR    CFG_BLE_MAX_SLEEP_DUR
#else
    #define QN_BLE_MAX_SLEEP_DUR    0x320 // 625us * 0x320 = 0.5s
#endif

#if (defined(CFG_T_VERSION))
    #define QN_PMU_VOLTAGE          1
#else
    #define QN_PMU_VOLTAGE          0
#endif

/// EACI external wakeup source
#if QN_EACI
#if (defined(CFG_EACI_GPIO_WAKEUP_QN_MCU))
    #define QN_EACI_GPIO_WAKEUP_QN_MCU         CFG_EACI_GPIO_WAKEUP_QN_MCU
#else
    #if (defined(CFG_BLE_SLEEP))
        #error "when EACI & allow sleep, need external wakeup source"
    #endif
#endif
#endif

/// Wakeup external mcu
#if ((defined(CFG_EACI_GPIO_WAKEUP_EX_MCU)) && (defined(CFG_HCI_UART)))
    #define QN_EACI_GPIO_WAKEUP_EX_MCU        CFG_EACI_GPIO_WAKEUP_EX_MCU
#endif

/// Menu option
#if (defined(CFG_DEMO_MENU))
    #define CFG_DBG_PRINT
    // Debug menu enable
    #define QN_DEMO_MENU            1
#endif

// #define QN_DEMO_AUTO             1

/// White List support
#if (defined(CFG_WL_SUPPORT))
#define QN_WL_SUPPORT               1
#endif

/// Link infomation option
#if (defined(CFG_LINK_INFO))
#define QN_LINK_INFO                1
#endif

/// Security On or Off
#if (defined(CFG_SECURITY_ON))
#define QN_SECURITY_ON              1
#endif

/// CSRK support
#if (defined(CFG_CSRK_SUPPORT) && defined(CFG_SECURITY_ON))
#define QN_CSRK_SUPPORT             1
#endif

/// Debug print option
#if (defined(CFG_DBG_PRINT))
    // QPRINTF enable
    #define QN_DBG_PRINT            1

    #if (defined(CFG_STD_PRINTF))
        #define QN_STD_PRINTF       1
    #else
        #define QN_STD_PRINTF       0
    #endif

    #if (defined(CFG_DBG_TRACE_MORE))
        // More Trace level
        #define QN_DBG_TRACE_MORE   1
    #else
        // Less Trace level
        #define QN_DBG_TRACE_MORE   0
    #endif
#else
    // QPRINTF disable
    #define QN_DBG_PRINT            0
    // Less Trace level
    #define QN_DBG_TRACE_MORE       0
#endif

/// Debug information
#if (defined(CFG_DBG_INFO))
    #define QN_DBG_INFO             1
#else
    #define QN_DBG_INFO             0
#endif

/// UART definitions
#define QN_DEBUG_UART               CFG_DEBUG_UART
#define QN_HCI_UART_RD              (hci_api)uart_read
#define QN_HCI_UART_WR              (hci_api)uart_write
#define QN_HCI_SPI_RD               (hci_api)spi_read
#define QN_HCI_SPI_WR               (hci_api)spi_write

#if (defined(CFG_WM_NP) || defined(CFG_WM_HCI) || defined(CFG_EACI) || defined(CFG_TEST_CTRL_PIN))
#if (defined(CFG_HCI_UART))
    #define QN_HCI_PORT             CFG_HCI_UART
    #define QN_HCI_RD               QN_HCI_UART_RD
    #define QN_HCI_WR               QN_HCI_UART_WR
#elif (defined(CFG_HCI_SPI))
    #define QN_HCI_PORT             CFG_HCI_SPI
    #define QN_HCI_RD               QN_HCI_SPI_RD
    #define QN_HCI_WR               QN_HCI_SPI_WR
#else
    #define QN_HCI_PORT             NULL
    #define QN_HCI_RD               NULL
    #define QN_HCI_WR               NULL
#endif
#else
    #define QN_HCI_PORT             NULL
    #define QN_HCI_RD               NULL
    #define QN_HCI_WR               NULL
#endif


/// Application role definitions
#define BLE_BROADCASTER             (defined(CFG_BROADCASTER))
#define BLE_OBSERVER                (defined(CFG_OBSERVER))
#define BLE_PERIPHERAL              (defined(CFG_PERIPHERAL) || defined(CFG_ALLROLES))
#define BLE_CENTRAL                 (defined(CFG_CENTRAL) || defined(CFG_ALLROLES))

#if (!BLE_BROADCASTER) && (!BLE_OBSERVER) && (!BLE_PERIPHERAL) && (!BLE_CENTRAL)
    #error "No application role defined"
#endif /* #if (!BLE_BROADCASTER) && (!BLE_OBSERVER) && (!BLE_PERIPHERAL) && (!BLE_CENTRAL) */

/// Maximum number of simultaneous connections
#if (BLE_CENTRAL)
    #define BLE_CONNECTION_MAX      CFG_CON
#elif BLE_PERIPHERAL
    #define BLE_CONNECTION_MAX      1
#else
    #define BLE_CONNECTION_MAX      1
#endif /* #if (BLE_CENTRAL) */

/// Address type of local device
#if (defined(CFG_ADDR_PUBLIC))
    #define QN_ADDR_TYPE            ADDR_PUBLIC
#elif (defined(CFG_ADDR_RAND))
    #define QN_ADDR_TYPE            ADDR_RAND
#else
    #define QN_ADDR_TYPE            ADDR_PUBLIC
#endif

/// Privacy enable
#if (defined(CFG_PRIV_EN))
    #define QN_PRIV_EN              1
#else
    #define QN_PRIV_EN              0
#endif

/// Set a true random seed to the ROM code
#if (defined(CFG_FW_SRAND))
    #define QN_FW_SRAND             1
#else
    #if (defined(CFG_ADDR_RAND))
        #define QN_FW_SRAND             1
    #else
        #define QN_FW_SRAND             0
    #endif
#endif

#if (defined(CFG_SVC_DISC))
    // Gatt Discoveried Service Used
    #define QN_SVC_DISC_USED        1
    // Gatt Discoveried Service Record number
    #define QN_GATT_MAX_HDL_NB      (3 * GATT_MAX_HDL_LIST)
#endif

/// SMP Security level and IO capbility definitions
#if (QN_SECURITY_ON)
    #if QN_DEMO_MENU
        #define QN_SEC_MODE_LEVEL       GAP_SEC1_NOAUTH_PAIR_ENC
        #define QN_IO_CAP_ABILITY       SMP_IO_CAP_KB_DISPLAY
    #elif QN_EACI
        #define QN_SEC_MODE_LEVEL       GAP_SEC1_NOAUTH_PAIR_ENC
        #define QN_IO_CAP_ABILITY       SMP_IO_CAP_NO_INPUT_NO_OUTPUT
    #else
        #define QN_SEC_MODE_LEVEL       GAP_SEC1_NOAUTH_PAIR_ENC
        #define QN_IO_CAP_ABILITY       SMP_IO_CAP_NO_INPUT_NO_OUTPUT
    #endif
#else
    #define QN_SEC_MODE_LEVEL       GAP_SEC1_NOAUTH_PAIR_ENC
    #define QN_IO_CAP_ABILITY       SMP_IO_CAP_NO_INPUT_NO_OUTPUT
#endif

/// TK
#define QN_SMP_TK   "111111"

/// IRK
#define QN_SMP_IRK  "\x01\x5F\xE8\xB4\x56\x07\x8E\x22\x18\xA6\x7C\xE1\xE4\xBA\x99\xA5"

/// CSRK
#define QN_SMP_CSRK "\x02\x45\x30\xDA\x3A\xFC\x81\x48\xF1\x0D\xAD\x2E\x91\x9D\x57\x7B"

/// LTK
#define QN_SMP_LTK  "\xBF\x01\xFB\x9D\x4E\xF3\xBC\x36\xD8\x74\xF5\x39\x41\x38\x68\x4C"

/// RAND
#define QN_SMP_RAND "\x00\x11\x22\x33\x44\x55\x66\x77"

/// EDIV
#define QN_SMP_EDIV 0x4321

/// BLE new features
#if (defined(CFG_MULTI_NOTIFICATION_IN_ONE_EVENT))
#define QN_MULTI_NOTIFICATION_IN_ONE_EVENT      1
#else
#define QN_MULTI_NOTIFICATION_IN_ONE_EVENT      0
#endif

#if (defined(CFG_READ_INDICATION))
#define QN_READ_INDICATION                      1
#else
#define QN_READ_INDICATION                      0
#endif

#if (defined(CFG_SLAVE_LATENCY_IMPROVEMENT))
#define QN_SLAVE_LATENCY_IMPROVEMENT            1
#else
#define QN_SLAVE_LATENCY_IMPROVEMENT            0
#endif

/// ATT DB,Testing and Qualification related flags
#if (BLE_CENTRAL || BLE_PERIPHERAL)

    ///Quintic private profile Client Role
    #if defined(CFG_PRF_QPPC)
        #define BLE_QPP_CLIENT      1
        #define TASK_QPPC           CFG_TASK_QPPC
    #else
        #define BLE_QPP_CLIENT      0
    #endif // defined(CFG_PRF_QPPC)

    ///Quintic private profile Server Role
    #if defined(CFG_PRF_QPPS)
        #define BLE_QPP_SERVER      1
        #define TASK_QPPS           CFG_TASK_QPPS
        #define QPPS_DB_SIZE        1400
    #else
        #define BLE_QPP_SERVER      0
        #define QPPS_DB_SIZE        0
    #endif // defined(CFG_PRF_QPPS)

    ///Health Thermometer Profile Collector Role
    #if defined(CFG_PRF_HTPC)
        #define BLE_HT_COLLECTOR    1
        #define TASK_HTPC           CFG_TASK_HTPC
    #else
        #define BLE_HT_COLLECTOR    0
    #endif // defined(CFG_PRF_HTPC)

    ///Health Thermometer Profile Thermometer Role
    #if defined(CFG_PRF_HTPT)
        #define BLE_HT_THERMOM      1
        #define TASK_HTPT           CFG_TASK_HTPT
        #define HTPT_DB_SIZE        244
    #else
        #define BLE_HT_THERMOM      0
        #define HTPT_DB_SIZE        0
    #endif // defined(CFG_PRF_HTPT)

    /// Proximity Profile Monitor Role
    #if defined(CFG_PRF_PXPM)
        #define BLE_PROX_MONITOR    1
        #define TASK_PROXM          CFG_TASK_PXPM
    #else
        #define BLE_PROX_MONITOR    0
    #endif // defined(CFG_PRF_PXPM)

    /// Proximity Profile Reporter Role
    #if defined(CFG_PRF_PXPR)
        #define BLE_PROX_REPORTER   1
        #define TASK_PROXR          CFG_TASK_PXPR
        #define PXPR_DB_SIZE        180
    #else
        #define BLE_PROX_REPORTER   0
        #define PXPR_DB_SIZE        0
    #endif // defined(CFG_PRF_PXPR)

    ///Find Me Profile Locator role
    #if defined(CFG_PRF_FMPL)
        #define BLE_FINDME_LOCATOR  1
        #define TASK_FINDL          CFG_TASK_FMPL
    #else
        #define BLE_FINDME_LOCATOR  0
    #endif // defined(CFG_PRF_FMPL)

    ///Find Me Profile Target role
    #if defined(CFG_PRF_FMPT)
        #define BLE_FINDME_TARGET   1
        #define TASK_FINDT          CFG_TASK_FMPT
        #define FMPT_DB_SIZE        60
    #else
        #define BLE_FINDME_TARGET   0
        #define FMPT_DB_SIZE        0
    #endif // defined(CFG_PRF_FMPT)

    ///Device Information Service Client Role
    #if defined(CFG_PRF_DISC)
        #define BLE_DIS_CLIENT      1
        #define TASK_DISC           CFG_TASK_DISC
    #else
        #define BLE_DIS_CLIENT      0
    #endif // defined(CFG_PRF_DISC)

    ///Device Information Service Server Role
    #if defined(CFG_PRF_DISS)
        #define BLE_DIS_SERVER      1
        #define TASK_DISS           CFG_TASK_DISS
        #define DISS_DB_SIZE        496
    #else
        #define BLE_DIS_SERVER      0
        #define DISS_DB_SIZE        0
    #endif // defined(CFG_PRF_DISS)

    ///Blood Pressure Profile Collector Role
    #if defined(CFG_PRF_BLPC)
        #define BLE_BP_COLLECTOR    1
        #define TASK_BLPC           CFG_TASK_BLPC
    #else
        #define BLE_BP_COLLECTOR    0
    #endif // defined(CFG_PRF_BLPC)

    ///Blood Pressure Profile Sensor Role
    #if defined(CFG_PRF_BLPS)
        #define BLE_BP_SENSOR       1
        #define TASK_BLPS           CFG_TASK_BLPS
        #define BLPS_DB_SIZE        192
    #else
        #define BLE_BP_SENSOR       0
        #define BLPS_DB_SIZE        0
    #endif // defined(CFG_PRF_BLPS)

    ///Time Profile Client Role
    #if defined(CFG_PRF_TIPC)
        #define BLE_TIP_CLIENT      1
        #define TASK_TIPC           CFG_TASK_TIPC
    #else
        #define BLE_TIP_CLIENT      0
    #endif // defined(CFG_PRF_TIPC)

    ///Time Profile Server Role
    #if defined(CFG_PRF_TIPS)
        #define BLE_TIP_SERVER      1
        #define TASK_TIPS           CFG_TASK_TIPS
        #define TIPS_DB_SIZE        312
    #else
        #define BLE_TIP_SERVER      0
        #define TIPS_DB_SIZE        0
    #endif // defined(CFG_PRF_TIPS)

    ///Heart Rate Profile Collector Role
    #if defined(CFG_PRF_HRPC)
        #define BLE_HR_COLLECTOR    1
        #define TASK_HRPC           CFG_TASK_HRPC
    #else
        #define BLE_HR_COLLECTOR    0
    #endif // defined(CFG_PRF_HRPC)

    ///Heart Rate Profile Sensor Role
    #if defined(CFG_PRF_HRPS)
        #define BLE_HR_SENSOR       1
        #define TASK_HRPS           CFG_TASK_HRPS
        #define HRPS_DB_SIZE        152
    #else
        #define BLE_HR_SENSOR       0
        #define HRPS_DB_SIZE        0
    #endif // defined(CFG_PRF_HRPS)

    ///Glucose Profile Client Role
    #if defined(CFG_PRF_GLPC)
        #define BLE_GL_COLLECTOR    1
        #define TASK_GLPC           CFG_TASK_GLPC
    #else
        #define BLE_GL_COLLECTOR    0
    #endif // defined(CFG_PRF_GLPC)

    ///Glucose Profile Server Role
    #if defined(CFG_PRF_GLPS)
        #define BLE_GL_SENSOR       1
        #define TASK_GLPS           CFG_TASK_GLPS
        #define GLPS_DB_SIZE        256
    #else
        #define BLE_GL_SENSOR       0
        #define GLPS_DB_SIZE        0
    #endif // defined(CFG_PRF_SCPPS)

    ///HID Profile Boot Host Role
    #if defined(CFG_PRF_HOGPBH)
        #define BLE_HID_BOOT_HOST    1
        #define TASK_HOGPBH          CFG_TASK_HOGPBH
    #else
        #define BLE_HID_BOOT_HOST    0
    #endif // defined(CFG_PRF_HOGPBH)

        ///HID Profile Report Host Role
    #if defined(CFG_PRF_HOGPRH)
        #define BLE_HID_REPORT_HOST  1
        #define TASK_HOGPRH          CFG_TASK_HOGPRH
    #else
        #define BLE_HID_REPORT_HOST  0
    #endif // defined(CFG_PRF_HOGPRH)

        ///HID Profile Device Role
    #if defined(CFG_PRF_HOGPD)
        #define BLE_HID_DEVICE      1
        #define TASK_HOGPD          CFG_TASK_HOGPD
        #define HOGPD_DB_SIZE       924  // one instance size
    #else
        #define BLE_HID_DEVICE      0
        #define HOGPD_DB_SIZE       0
    #endif // defined(CFG_PRF_HOGPD)

    ///Scan Parameter Profile Client Role
    #if defined(CFG_PRF_SCPPC)
        #define BLE_SP_CLIENT       1
        #define TASK_SCPPC          CFG_TASK_SCPPC
    #else
        #define BLE_SP_CLIENT       0
    #endif // defined(CFG_PRF_SCPPC)

    ///Scan Parameter Profile Server Role
    #if defined(CFG_PRF_SCPPS)
        #define BLE_SP_SERVER       1
        #define TASK_SCPPS          CFG_TASK_SCPPS
        #define SCPPS_DB_SIZE       112
    #else
        #define BLE_SP_SERVER       0
        #define SCPPS_DB_SIZE       0
    #endif // defined(CFG_PRF_SCPPS)

    ///Battery Service Client Role
    #if defined(CFG_PRF_BASC)
        #define BLE_BATT_CLIENT     1
        #define TASK_BASC           CFG_TASK_BASC
    #else
        #define BLE_BATT_CLIENT     0
    #endif // defined(CFG_PRF_BASC)

    ///Battery Service Server Role
    #if defined(CFG_PRF_BASS)
        #define BLE_BATT_SERVER     1
        #define TASK_BASS           CFG_TASK_BASS
        #define BASS_DB_SIZE        (96*2) // the size of every instance is 95
    #else
        #define BLE_BATT_SERVER     0
        #define BASS_DB_SIZE        0
    #endif // defined(CFG_PRF_BASS)

    ///Alert Notification Client Role
    #if defined(CFG_PRF_ANPC)
        #define BLE_AN_CLIENT       1
        #define TASK_ANPC           CFG_TASK_ANPC
    #else
        #define BLE_AN_CLIENT       0
    #endif // defined(CFG_PRF_ANPC)

    ///Alert Notification Server Role
    #if defined(CFG_PRF_ANPS)
        #define BLE_AN_SERVER       1
        #define TASK_ANPS           CFG_TASK_ANPS
        #define ANPS_DB_SIZE        240
    #else
        #define BLE_AN_SERVER       0
        #define ANPS_DB_SIZE        0
    #endif // defined(CFG_PRF_ANPS)

    ///Cycling Speed and Cadence Collector Role
    #if defined(CFG_PRF_CSCPC)
        #define BLE_CSC_COLLECTOR   1
        #define TASK_CSCPC          CFG_TASK_CSCPC
    #else
        #define BLE_CSC_COLLECTOR   0
    #endif // defined(CFG_PRF_CSCPC)

    ///Cycling Speed and Cadence Sensor Role
    #if defined(CFG_PRF_CSCPS)
        #define BLE_CSC_SENSOR      1
        #define TASK_CSCPS          CFG_TASK_CSCPS
        #define CSCPS_DB_SIZE       152
    #else
        #define TASK_CSCPS          0
        #define CSCPS_DB_SIZE       0
    #endif // defined(CFG_PRF_CSCPS)

    ///Phone Alert Status Client Role
    #if defined(CFG_PRF_PASPC)
        #define BLE_PAS_CLIENT      1
        #define TASK_PASPC          CFG_TASK_PASPC
    #else
        #define BLE_PAS_CLIENT      0
    #endif // defined(CFG_PRF_PASPC)

    ///Phone Alert Status Server Role
    #if defined(CFG_PRF_PASPS)
        #define BLE_PAS_SERVER      1
        #define TASK_PASPS          CFG_TASK_PASPS
        #define PASPS_DB_SIZE       156
    #else
        #define BLE_PAS_SERVER      0
        #define PASPS_DB_SIZE       0
    #endif // defined(CFG_PRF_PASPS)

    ///Running Speed and Cadence Collector Role
    #if defined(CFG_PRF_RSCPC)
        #define BLE_RSC_COLLECTOR   1
        #define TASK_RSCPC          CFG_TASK_RSCPC
    #else
        #define BLE_RSC_COLLECTOR   0
    #endif // defined(CFG_PRF_RSCPC)

    ///Running Speed and Cadence Sensor Role
    #if defined(CFG_PRF_RSCPS)
        #define BLE_RSC_SENSOR      1
        #define TASK_RSCPS          CFG_TASK_RSCPS
        #define RSCPS_DB_SIZE       148
    #else
        #define BLE_RSC_SENSOR      0
        #define RSCPS_DB_SIZE       0
    #endif // defined(CFG_PRF_RSCPS)

    ///OTA Server Role
    #if defined(CFG_PRF_OTAS)
        #define BLE_OTA_SERVER      1
        #define TASK_OTAS           CFG_TASK_OTAS
        #define OTAS_DB_SIZE        1200
        //#define ENAB_OTAS_APP_CTRL
        //#define ENAB_OTAS_SEND_DATA
        //#define ENAB_OTAS_SET_UUID
    #else
        #define BLE_OTA_SERVER      0
        #define OTAS_DB_SIZE        0
    #endif // defined(CFG_PRF_OTAS)

    ///ANCS Client Role
    #if defined(CFG_PRF_ANCSC)
        #define BLE_ANCS_NC      1
        #define TASK_ANCSC       CFG_TASK_ANCSC
    #else
        #define BLE_ANCS_NC      0
    #endif // defined(CFG_PRF_ANCSC)

    //Force ATT parts depending on profile roles or compile options
    /// Attribute Client
    #if BLE_RSC_COLLECTOR || BLE_PAS_CLIENT || BLE_CSC_COLLECTOR || BLE_AN_CLIENT || BLE_PROX_MONITOR || BLE_FINDME_LOCATOR || BLE_HT_COLLECTOR || BLE_BP_COLLECTOR || BLE_HR_COLLECTOR || BLE_DIS_CLIENT || BLE_TIP_CLIENT || BLE_SP_CLIENT || BLE_BATT_CLIENT || (defined(CFG_ATTC))
        #define BLE_ATTC            1
    #else
        #define BLE_ATTC            0
    #endif // BLE_PROX_MONITOR || BLE_FINDME_LOCATOR || BLE_HT_COLLECTOR || BLE_BP_COLLECTOR || BLE_HR_COLLECTOR || BLE_DIS_CLIENT || BLE_TIP_CLIENT || BLE_SP_CLIENT || BLE_BATT_CLIENT || (defined(CFG_ATTC))

    /// Attribute Server
    #if BLE_RSC_SENSOR || BLE_PAS_SERVER || BLE_CSC_SENSOR || BLE_AN_SERVER || BLE_PROX_REPORTER || BLE_FINDME_TARGET || BLE_HT_THERMOM || BLE_BP_SENSOR || BLE_TIP_SERVER || BLE_HR_SENSOR || BLE_DIS_SERVER || BLE_ACCEL || BLE_SP_SERVER || BLE_BATT_SERVER || defined(CFG_ATTS)
        #define BLE_ATTS            1
    #else
        #define BLE_ATTS            0
    #endif // BLE_PROX_REPORTER || BLE_FINDME_TARGET || BLE_HT_THERMOM || BLE_BP_SENSOR || BLE_TIP_SERVER || BLE_HR_SENSOR || BLE_DIS_SERVER || BLE_ACCEL || BLE_SP_SERVER || BLE_BATT_SERVER || defined(CFG_ATTS)

    //Force ATT parts depending on profile roles or compile options
    /// Attribute Client
    #if (defined(CFG_ATTC))
        #define BLE_ATTC            1
    #else
        #define BLE_ATTC            0
    #endif // (defined(CFG_ATTC))

    /// Attribute Server
    #if defined(CFG_ATTS)
        #define BLE_ATTS            1
    #else
        #define BLE_ATTS            0
    #endif // defined(CFG_ATTS)

#elif (BLE_OBSERVER || BLE_BROADCASTER)

    /// Proximity Profile Monitor Role
    #define BLE_PROX_MONITOR        0
    /// Proximity Profile Reporter Role
    #define BLE_PROX_REPORTER       0
    ///Find Me Profile Locator role
    #define BLE_FINDME_LOCATOR      0
    ///Find Me Profile Target role
    #define BLE_FINDME_TARGET       0
    ///Health Thermometer Profile Collector Role
    #define BLE_HT_COLLECTOR        0
    ///Health Thermometer Profile Thermometer Role
    #define BLE_HT_THERMOM          0
    ///Blood Pressure Profile Collector Role
    #define BLE_BP_COLLECTOR        0
    ///Blood Pressure Profile Sensor Role
    #define BLE_BP_SENSOR           0
    ///Heart Rate Profile Collector Role
    #define BLE_HR_COLLECTOR        0
    ///Heart Rate Profile Sensor Role
    #define BLE_HR_SENSOR           0
    ///Time Profile Client Role
    #define BLE_TIP_CLIENT          0
    ///Time Profile Server Role
    #define BLE_TIP_SERVER          0
    /// Accelerometer Profile
    #define BLE_ACCEL               0
    /// Device Information Service Client Role
    #define BLE_DIS_CLIENT          0
    /// Device Information Service Server Role
    #define BLE_DIS_SERVER          0
    ///Alert Notification Client Role
    #define BLE_AN_CLIENT           0
    ///Alert Notification Server Role
    #define BLE_AN_SERVER           0
    ///Cycling Speed and Cadence Collector Role
    #define BLE_CSC_COLLECTOR       0
    ///Cycling Speed and Cadence Sensor Role
    #define TASK_CSCPS              0
    ///Phone Alert Status Client Role
    #define BLE_PAS_CLIENT          0
    ///Phone Alert Status Server Role
    #define BLE_PAS_SERVER          0
    ///Running Speed and Cadence Collector Role
    #define BLE_RSC_COLLECTOR       0
    ///Running Speed and Cadence Sensor Role
    #define BLE_RSC_SENSOR          0

    //Force ATT parts to 0
    #define BLE_ATTC                0
    /// Attribute Server
    #define BLE_ATTS                0
#endif //BLE_CENTRAL || BLE_PERIPHERAL

/// Size of the heap
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /// some heap must be reserved for attribute database
    #if (BLE_ATTS || BLE_ATTC)
        #define GAP_DB_SIZE         (232)
        #define GATT_DB_SIZE        (60)
        #define BLE_DB_SIZE         (GAP_DB_SIZE+GATT_DB_SIZE+QPPS_DB_SIZE+OTAS_DB_SIZE+HTPT_DB_SIZE+PXPR_DB_SIZE+FMPT_DB_SIZE+DISS_DB_SIZE+ BLPS_DB_SIZE+TIPS_DB_SIZE+HRPS_DB_SIZE+HOGPD_DB_SIZE+SCPPS_DB_SIZE+BASS_DB_SIZE+GLPS_DB_SIZE+RSCPS_DB_SIZE+PASPS_DB_SIZE+CSCPS_DB_SIZE+ANPS_DB_SIZE)
    #else
        #define BLE_DB_SIZE         (0)
    #endif /* (BLE_ATTS || BLE_ATTC) */

        #if defined(CFG_WM_NP)
            #define BLE_HEAP_SIZE            20480
        #else
            #define BLE_HEAP_SIZE           (BLE_DB_SIZE + 512 + 256 * BLE_CONNECTION_MAX)
        #endif
#else
    #define BLE_HEAP_SIZE           (800)
#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

/// @} APP_CONFIG

#endif // _APP_CONFIG_H_

