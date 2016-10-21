/**
 ****************************************************************************************
 *
 * @file app_env.h
 *
 * @brief Application Header File
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_ENV_H_
#define _APP_ENV_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "co_bt.h"
#include "co_error.h"

#include "app_config.h"
#include "app_sys.h"
#include "app_gap.h"
#include "app_smp.h"
#include "app_gatt.h"
#include "app_task.h"
#include "app_util.h"
#include "app_printf.h"

#if !QN_WORK_MODE
#include "nvds.h"
#endif

#if BLE_HT_COLLECTOR
#include "app_htpc.h"
#endif
#if BLE_BP_COLLECTOR
#include "app_blpc.h"
#endif
#if BLE_HR_COLLECTOR
#include "app_hrpc.h"
#endif
#if BLE_GL_COLLECTOR
#include "app_glpc.h"
#endif
#if BLE_FINDME_LOCATOR
#include "app_findl.h"
#endif
#if BLE_PROX_MONITOR
#include "app_proxm.h"
#endif
#if BLE_TIP_CLIENT
#include "app_tipc.h"
#endif
#if BLE_HID_BOOT_HOST
#include "app_hogpbh.h"
#endif
#if BLE_HID_REPORT_HOST
#include "app_hogprh.h"
#endif
#if BLE_SP_CLIENT
#include "app_scppc.h"
#endif
#if BLE_DIS_CLIENT
#include "app_disc.h"
#endif
#if BLE_BATT_CLIENT
#include "app_basc.h"
#endif
#if BLE_CSC_COLLECTOR
#include "app_cscpc.h"
#endif
#if BLE_RSC_COLLECTOR
#include "app_rscpc.h"
#endif
#if BLE_PAS_CLIENT
#include "app_paspc.h"
#endif
#if BLE_AN_CLIENT
#include "app_anpc.h"
#endif
#if BLE_HT_THERMOM
#include "app_htpt.h"
#endif
#if BLE_BP_SENSOR
#include "app_blps.h"
#endif
#if BLE_HR_SENSOR
#include "app_hrps.h"
#endif
#if BLE_PROX_REPORTER
#include "app_proxr.h"
#endif
#if BLE_FINDME_TARGET
#include "app_findt.h"
#endif
#if BLE_TIP_SERVER
#include "app_tips.h"
#endif
#if BLE_SP_SERVER
#include "app_scpps.h"
#endif
#if BLE_DIS_SERVER
#include "app_diss.h"
#endif
#if BLE_BATT_SERVER
#include "app_bass.h"
#endif
#if BLE_GL_SENSOR
#include "app_glps.h"
#endif
#if BLE_HID_DEVICE
#include "app_hogpd.h"
#endif
#if BLE_CSC_SENSOR
#include "app_cscps.h"
#endif
#if BLE_RSC_SENSOR
#include "app_rscps.h"
#endif
#if BLE_PAS_SERVER
#include "app_pasps.h"
#endif
#if BLE_AN_SERVER
#include "app_anps.h"
#endif

#if BLE_QPP_CLIENT
#include "app_qppc.h"
#include "qppc_task.h"
#endif
#if BLE_QPP_SERVER
#include "app_qpps.h"
#endif

#if BLE_OTA_SERVER
#include "otas_task.h"
#include "app_otas_task.h"
#endif

#if BLE_ANCS_NC
#include "app_ancsc.h"
#include "app_ancsc_task.h"
#endif
    
#if QN_DEMO_MENU
#include "app_menu.h"
#endif

#include "usr_design.h"

#if QN_EACI
#include "app_eaci.h"
#include "app_eaci_task_msg.h"
#include "app_eaci_trans.h"
#endif

#if QN_EAPI
#include "app_eapi.h"
#endif

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
#define APP_IDX_MAX                                 0x01

#define APP_MAX_BONDED_DEVICE_NUMBER                1

// The TAG value after 100 reserved for application
// Store bonded number
#define APP_NVDS_DB_COUNT_TAG                       (50)
// Store bonded information
#define APP_NVDS_DB_START_TAG                       (APP_NVDS_DB_COUNT_TAG + 1)
#define APP_NVDS_DB_END_TAG                         (APP_NVDS_DB_COUNT_TAG + APP_MAX_BONDED_DEVICE_NUMBER)

// Aligned to 4 bytes
#define BONDED_DB_SIZE  ((sizeof(struct app_bonded_info) * APP_MAX_BONDED_DEVICE_NUMBER - 1) / sizeof(uint32_t) + 1)

// Invalid index for profile client
#define APP_INVALID_INDEX                           (0xFF)

/// Service Record Structure
struct app_svc_record
{
    /// attribute handle
    uint16_t attr_hdl;
};

/// Pair Information Structure
struct app_pair_info
{
    struct smp_key irk;
#if (QN_CSRK_SUPPORT)
    struct smp_key csrk;
#endif
    struct smp_key ltk;
    ///EDIV (=0 if not sending an LTK)
    uint16_t ediv;
    ///Random number (=0 if not sending an LTK)
    struct rand_nb rand_nb;
};

/// Stored Pair Information Structure
struct app_bonded_info
{
    uint8_t sec_prop;
    uint8_t peer_distribute_keys;
    uint8_t addr_type; 
    /// status of the device record
    struct bd_addr peer_addr;
    struct app_pair_info pair_info;
};

/// Connected Device Record Structure
struct app_dev_record
{
    /// Temporary store bonded information
    struct app_bonded_info bonded_info;

    /// discoveried service record list
#if QN_SVC_DISC_USED
    uint8_t nb_svc;
    struct app_svc_record svc_rec[QN_GATT_MAX_HDL_NB];
#endif

    uint16_t conhdl;
    bool free;
};

#if (defined(CFG_EACI))
struct device_name_set
{
    uint8_t namelen;
    uint8_t name[0x14];
};
#endif

/// Application Environment Structure
struct app_env_tag
{
#if (BLE_PERIPHERAL || BLE_BROADCASTER)
    // Service setup Flag
    uint16_t srv_flag;
    // Advertising data
    uint8_t adv_data[ADV_DATA_LEN];
    // Scan Response data
    uint8_t scanrsp_data[SCAN_RSP_DATA_LEN];
#endif

#if (QN_SECURITY_ON)
    // Security Level
    uint8_t sec_lvl_set;
    // IO capabilities
    uint8_t iocap_set;
    // index of TK request indication
    uint8_t tk_idx;
    // TK type
    uint8_t tk_type;

    // TK
    uint8_t tk[7];
    // IRK
    uint8_t irk[KEY_LEN];
#if (QN_CSRK_SUPPORT)
    // CSRK
    uint8_t csrk[KEY_LEN];
#endif
    // LTK
    uint8_t ltk[KEY_LEN];
    // RAND
    uint8_t rand[RAND_NB_LEN];
    // EDIV
    uint16_t ediv;
    // Bonded flag
    uint8_t bond_flag;
    // Bonded device Count
    uint8_t bonded_count;
    // Pointer to Bonded Database
    struct app_bonded_info *bonded_info;
    // Bonded Database
    uint32_t bonded_db[BONDED_DB_SIZE]; 
#endif

#if (BLE_CENTRAL || BLE_OBSERVER)
    // Selected Dev Idx
    uint8_t select_idx;
    // Inquiried Device Record
    uint8_t inq_idx;
    uint8_t addr_type[BLE_CONNECTION_MAX];
    struct bd_addr inq_addr[BLE_CONNECTION_MAX];
#endif
    // Local role
    uint8_t role;
    // Record requested IRK position
    uint8_t irk_pos;
    // Connected Device Count
    uint8_t cn_count;
    // Connected Device Record
    struct app_dev_record dev_rec[BLE_CONNECTION_MAX];

#if BLE_AN_CLIENT
    struct app_anpc_env_tag anpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_AN_SERVER
    struct app_anps_env_tag anps_ev;
#endif

#if BLE_HT_COLLECTOR
    struct app_htpc_env_tag htpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_HT_THERMOM
    struct app_htpt_env_tag htpt_ev;
#endif

#if BLE_BP_COLLECTOR
    struct app_blpc_env_tag blpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_BP_SENSOR
    struct app_blps_env_tag blps_ev;
#endif

#if BLE_HR_COLLECTOR
    struct app_hrpc_env_tag hrpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_HR_SENSOR
    struct app_hrps_env_tag hrps_ev;
#endif

#if BLE_FINDME_LOCATOR
    struct app_findl_env_tag findl_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_FINDME_TARGET
    struct app_findt_env_tag findt_ev;
#endif

#if BLE_PROX_MONITOR
    struct app_proxm_env_tag proxm_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_PROX_REPORTER
    struct app_proxr_env_tag proxr_ev;
#endif

#if BLE_SP_CLIENT
    struct app_scppc_env_tag scppc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_SP_SERVER
    struct app_scpps_env_tag scpps_ev;
#endif

#if BLE_TIP_CLIENT
    struct app_tipc_env_tag tipc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_TIP_SERVER
    struct app_tips_env_tag tips_ev;
#endif

#if BLE_GL_COLLECTOR
    struct app_glpc_env_tag glpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_GL_SENSOR
    struct app_glps_env_tag glps_ev;
#endif

#if BLE_HID_BOOT_HOST
    struct app_hogpbh_env_tag hogpbh_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_HID_REPORT_HOST
    struct app_hogprh_env_tag hogprh_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_HID_DEVICE
    struct app_hogpd_env_tag hogpd_ev;
#endif

#if BLE_DIS_CLIENT
    struct app_disc_env_tag disc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_DIS_SERVER
    struct app_diss_env_tag diss_ev;
#endif

#if BLE_BATT_CLIENT
    struct app_basc_env_tag basc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_BATT_SERVER
    struct app_bass_env_tag bass_ev;
#endif

#if BLE_CSC_COLLECTOR
    struct app_cscpc_env_tag cscpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_CSC_SENSOR
    struct app_cscps_env_tag cscps_ev;
#endif

#if BLE_RSC_COLLECTOR
    struct app_rscpc_env_tag rscpc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_RSC_SENSOR
    struct app_rscps_env_tag rscps_ev;
#endif

#if BLE_PAS_CLIENT
    struct app_paspc_env_tag paspc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_PAS_SERVER
    struct app_pasps_env_tag pasps_ev;
#endif

#if BLE_QPP_CLIENT
    struct app_qppc_env_tag qppc_ev[BLE_CONNECTION_MAX];
#endif

#if BLE_QPP_SERVER
    struct app_qpps_env_tag qpps_ev;
#endif

#if (BLE_ANCS_NC)
    struct app_ancsc_env_tag ancsc_ev[BLE_CONNECTION_MAX];
#endif

#if (BLE_OTA_SERVER)
    #ifdef ENAB_OTAS_SET_UUID
    uint8_t app_otas_uuid_flag;
#endif
#endif


#if QN_DEMO_MENU
    uint8_t menu_id;
    uint8_t input[0x0F];
#endif
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// Application Environment
extern struct app_env_tag app_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Initialize the application task environment.
 *
 ****************************************************************************************
 */
void app_init(void);


/*
 ****************************************************************************************
 * @brief Callback function of MENU.
 *
 ****************************************************************************************
 */
#if !QN_DEMO_MENU
#define app_menu_cb(m, p)
#endif

/// @} APP

#endif // APP_H_
