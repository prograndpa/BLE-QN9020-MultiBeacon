/**
 ****************************************************************************************
 *
 * @file app_env.c
 *
 * @brief Application C file
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
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_env_tag app_env;

/*
 ****************************************************************************************
 * @brief Iitiate Alert Notification Server enviroment
 * @description
 * Iitiate Alert Notification Server
 ****************************************************************************************
 */
#if BLE_AN_SERVER
static void app_anps_init(void)
{
    app_anps_env->supp_new_alert_cat = 0x03ff; // Bit Mask 0 0xff,Bit Mask 1 0x03
    app_anps_env->supp_unread_alert_cat = 0x03ff;
    app_anps_env->ntf_cfg = PRF_CLI_STOP_NTFIND;
    app_anps_env->alert_category_id = 0;
    app_anps_env->unread_alert_category_id = 0;
    app_anps_env->conhdl = 0xFFFF;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the battery service server enviroment - at initiation
 *
 ****************************************************************************************
 */
#if BLE_BATT_SERVER
static void app_bass_init(void)
{
    app_bass_env->bas_nb = BASS_NB_BAS_INSTANCES_MAX;
    for (uint8_t i = 0; i < app_bass_env->bas_nb; i++)
    {
        app_bass_env->features[i] = BAS_BATT_LVL_NTF_SUP;
        app_bass_env->batt_level_ntf_cfg[i] = PRF_CLI_STOP_NTFIND;  // Battery Level Notification Configurations
        app_bass_env->old_batt_lvl[i] = 80;                         // Last Battery Level
        app_bass_env->current_batt_lvl[i] = 90;                     // Current Battery Level

        // When a device has more than one instance of the Battery service, 
        // each Battery Level characteristic shall include a Characteristic Presentation Format descriptor
        if (app_bass_env->bas_nb > 1)
        {
            app_bass_env->batt_level_pres_format[i].unit = 0x27AD;      // percenty
            app_bass_env->batt_level_pres_format[i].description = i + 1;// 1 means first
            app_bass_env->batt_level_pres_format[i].format = 0x08;      // uint32 format
            app_bass_env->batt_level_pres_format[i].exponent = 0;
            app_bass_env->batt_level_pres_format[i].namespace = 1;      // Bluetooth SIG Assigned Numbers
        }
    }
    app_bass_env->conhdl = 0xFFFF;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate blood pressure service enviroment   *//**
 * @description
 *  Initiate blood pressure service enviroment.
 ****************************************************************************************
 */
#if BLE_BP_SENSOR
static void app_blps_init(void)
{
    app_blps_env->features = BLPS_INTM_CUFF_PRESS_SUP;
    app_blps_env->evt_cfg = 0;
    app_blps_env->conhdl = 0xFFFF;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate device information service enviroment
 * @description
 *  Initiate device information service enviroment
 ****************************************************************************************
 */
#if BLE_DIS_SERVER
static void app_diss_init(void)
{
    app_diss_env->features = DIS_MANUFACTURER_NAME_CHAR_SUP |
                            DIS_MODEL_NB_STR_CHAR_SUP       |
                            DIS_SERIAL_NB_STR_CHAR_SUP      |
                            DIS_HARD_REV_STR_CHAR_SUP       |
                            DIS_FIRM_REV_STR_CHAR_SUP       |    
                            DIS_SW_REV_STR_CHAR_SUP         |
                            DIS_SYSTEM_ID_CHAR_SUP          |
                            DIS_IEEE_CHAR_SUP               |
                            DIS_PNP_ID_CHAR_SUP;
}
#endif

/*
 ****************************************************************************************
 * @brief Iitiate find me target enviroment
 * @description
 * Iitiate find me target enviroment
 ****************************************************************************************
 */
#if BLE_FINDME_TARGET
static void app_findt_init(void)
{
   app_findt_env->conhdl = 0xFFFF;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the Glucose Profile Sensor enviroment - at initiation
 *
 ****************************************************************************************
 */
#if BLE_GL_SENSOR
static void app_glps_init(void)
{
    memset(&app_glps_env->racp_req, 0x00, sizeof(struct glp_racp_req));
    app_glps_env->conhdl = 0xFFFF;
    app_glps_env->evt_cfg = 0;
    app_glps_env->records_idx = 0;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the HID OVER GATT Profile device role enviroment - at initiation
 *
 ****************************************************************************************
 */
#if BLE_HID_DEVICE
static void app_hogpd_init(void)
{
    app_hogpd_env->conhdl = 0xFFFF;
    app_hogpd_env->hids_nb = 1;
    app_hogpd_env->ntf_sending = 0;
#if 1
    app_hogpd_env->features[0].svc_features = HOGPD_CFG_MOUSE |
                                              HOGPD_CFG_PROTO_MODE |
                                              #if BLE_BATT_SERVER
                                              HOGPD_CFG_MAP_EXT_REF |
                                              #endif
                                              HOGPD_CFG_BOOT_MOUSE_WR;
    app_hogpd_env->features[0].report_nb = 1;
    app_hogpd_env->features[0].report_char_cfg[0] = HOGPD_CFG_REPORT_IN | HOGPD_CFG_REPORT_WR;
    app_hogpd_env->proto_mode[0] = HOGP_REPORT_PROTOCOL_MODE;
#else
    app_hogpd_env->features[0].svc_features = HOGPD_CFG_KEYBOARD |
                                              HOGPD_CFG_PROTO_MODE |
                                              #if BLE_BATT_SERVER
                                              HOGPD_CFG_MAP_EXT_REF |
                                              #endif
                                              HOGPD_CFG_BOOT_KB_WR;
    app_hogpd_env->features[0].report_nb = 1;
    app_hogpd_env->features[0].report_char_cfg[0] = HOGPD_CFG_REPORT_IN | HOGPD_CFG_REPORT_WR;
    app_hogpd_env->proto_mode[0] = HOGP_REPORT_PROTOCOL_MODE;   
#endif
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the heart rate enviroment - at initiation
 * @description
 *  Initiate the heart rate enviroment - at initiation.
 ****************************************************************************************
 */
#if BLE_HR_SENSOR
static void app_hrps_init(void)
{
    app_hrps_env->features = HRPS_BODY_SENSOR_LOC_CHAR_SUP | HRPS_ENGY_EXP_FEAT_SUP;
    app_hrps_env->conhdl = 0xFFFF;
    app_hrps_env->energy_expended = 0;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the health thermometer server service enviroment - at initiation *//**
 * @description
 * Initiate the health thermometer server service enviroment - at initiation.
 ****************************************************************************************
 */
#if BLE_HT_THERMOM
static void app_htpt_init(void)
{
    app_htpt_env->features = HTPT_TEMP_TYPE_CHAR_SUP    |   // Indicate if Temperature Type Char. is supported
                            HTPT_INTERM_TEMP_CHAR_SUP   |   // Indicate if Intermediate Temperature Char. is supported
                            HTPT_MEAS_INTV_CHAR_SUP     |   // Indicate if Measurement Interval Char. is supported
                            HTPT_MEAS_INTV_IND_SUP      |   // Indicate if Measurement Interval Char. supports indications
                            HTPT_MEAS_INTV_WR_SUP;          // Indicate if Measurement Interval Char. is writable
    app_htpt_env->conhdl = 0xffff;
    app_htpt_env->meas_intv = APP_HTPT_MEAS_INTV;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the proximity reporter enviroment - at initiation   *//**
 * @description
 * Initiate the proximity reporter enviroment - at initiation.
 ****************************************************************************************
 */
#if BLE_PROX_REPORTER
static void app_proxr_init(void)
{
    // find me profile also has a IAS, so there is only one IAS instance in one device.
#if BLE_FINDME_TARGET
    app_proxr_env->features = 0;
#else
    app_proxr_env->features = PROXR_IAS_TXPS_SUP;
#endif
    app_proxr_env->conhdl = 0xFFFF;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the scan parameters profile server enviroment - at initiation   *//**
 * @description 
 * Initiate the scan parameters profile server enviroment - at initiation
 ****************************************************************************************
 */
#if BLE_SP_SERVER
static void app_scpps_init(void)
{
    app_scpps_env->features = SCPPS_SCAN_REFRESH_CHAR_SUP;
    app_scpps_env->conhdl = 0xFFFF;
    app_scpps_env->ntf_sending = false;
}
#endif

/*
 ****************************************************************************************
 * @brief Initiate the time server enviroment - at initiation  *//**
 * @description   
 *  Initiate the time server enviroment - at initiation
 ****************************************************************************************
 */
#if BLE_TIP_SERVER
static void app_tips_init(void)
{
    app_tips_env->features = TIPS_CTS_LOC_TIME_INFO_SUP |
                             TIPS_CTS_REF_TIME_INFO_SUP |
                             TIPS_NDCS_SUP              |
                             TIPS_RTUS_SUP;
    app_tips_env->conhdl = 0xFFFF;
}
#endif

static void app_gap_init(void)
{  
    // Set White List to NVDS, TODO

#if (BLE_PERIPHERAL)
    // Initialize Advertising data
    app_set_adv_data(GAP_GEN_DISCOVERABLE);
    // Initialize Scan Response data
    app_set_scan_rsp_data(0);
#endif
}

#define TK_LEN        6
#if (QN_SECURITY_ON)
static void app_smp_init(void)
{
#if !QN_WORK_MODE
    nvds_tag_len_t length = 0;
    
    // Get the stored TK type from NVDS
    length = 1;
    if (NVDS_OK != nvds_get(NVDS_TAG_TK_TYPE, &length, (uint8_t *)&app_env.tk_type))
    {
        // Fix TK used
        app_env.tk_type = 0;
    }
    // Get the stored TK from NVDS
    if (app_env.tk_type == 0)
    {
        length = TK_LEN;
        if (NVDS_OK != nvds_get(NVDS_TAG_TK_KEY, &length, app_env.tk))
            memcpy(app_env.tk, QN_SMP_TK, TK_LEN);
    }
    // Get the stored IRK from NVDS
    length = KEY_LEN;
    if (NVDS_OK != nvds_get(NVDS_TAG_IRK_KEY, &length, app_env.irk))
        memcpy(app_env.irk, QN_SMP_IRK, KEY_LEN);
    // Get the stored CSRK from NVDS
#if (QN_CSRK_SUPPORT)
    length = KEY_LEN;
    if (NVDS_OK != nvds_get(NVDS_TAG_CSRK_KEY, &length, app_env.csrk))
        memcpy(app_env.csrk, QN_SMP_CSRK, KEY_LEN);
#endif
    // Get the stored LTK from NVDS
    length = KEY_LEN;
    if (NVDS_OK != nvds_get(NVDS_TAG_LTK_KEY, &length, app_env.ltk))
        memcpy(app_env.ltk, QN_SMP_LTK, KEY_LEN);
    // Get the paired device information
    length = 1;
    if (NVDS_OK != nvds_get(APP_NVDS_DB_COUNT_TAG, &length, &app_env.bonded_count))
    {
        app_env.bonded_count = 0;
    }
    length = sizeof(struct app_bonded_info);
    for (uint8_t i = 0; i < app_env.bonded_count; i++)
    {
        if (NVDS_OK != nvds_get(APP_NVDS_DB_START_TAG + i, &length, (uint8_t *)(app_env.bonded_info + i)))
        {
            app_env.bonded_count = i;
            break;
        }
    }
#else
    // Fix TK not used
    app_env.tk_type = 1; 
    // Use default TK
    memcpy(app_env.tk, QN_SMP_TK, TK_LEN);
    // Use default IRK
    memcpy(app_env.irk, QN_SMP_IRK, KEY_LEN);
    // Use default CSRK
#if (QN_CSRK_SUPPORT)
    memcpy(app_env.csrk, QN_SMP_CSRK, KEY_LEN);
#endif
    // Use default LTK
    memcpy(app_env.ltk, QN_SMP_LTK, KEY_LEN);
#endif
    // RAND
    memcpy(app_env.rand, QN_SMP_RAND, RAND_NB_LEN);
    // EDIV
    app_env.ediv = QN_SMP_EDIV;

    // Default Security Level setting
    app_env.sec_lvl_set = QN_SEC_MODE_LEVEL;
    // Default IO Capbility setting
    app_env.iocap_set = QN_IO_CAP_ABILITY;
}
#endif

static void app_env_init(void)
{
    memset(&app_env, 0, sizeof(app_env));
    for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
        app_env.dev_rec[idx].free = true;
        app_env.dev_rec[idx].conhdl = 0xFFFF;
    }
    // Set White List
    app_gap_init();
    // Set Local Keys, Security Level and IO Capbility
#if (QN_SECURITY_ON)
    app_env.bonded_info = (struct app_bonded_info *)app_env.bonded_db;
    app_smp_init();
#endif
    // Set Service setup inital value
#if (BLE_PERIPHERAL)
    app_env.srv_flag = app_get_local_service_flag();
#endif
}

/**
 ****************************************************************************************
 * @brief Initialize the application task environment.
 *
 ****************************************************************************************
 */
void app_init(void)
{
    app_env_init();

    // Register APP task into kernel
#if !QN_WORK_MODE
    task_app_desc_register();
#endif    
    ke_state_set(TASK_APP, APP_INIT);

#if BLE_AN_SERVER
    app_anps_init();
#endif
#if BLE_HT_THERMOM
    app_htpt_init();
#endif
#if BLE_BP_SENSOR
    app_blps_init();
#endif
#if BLE_DIS_SERVER
    app_diss_init();
#endif
#if BLE_FINDME_TARGET
    app_findt_init();
#endif
#if BLE_HR_SENSOR
    app_hrps_init();
#endif
#if BLE_PROX_REPORTER
    app_proxr_init();
#endif
#if BLE_TIP_SERVER
    app_tips_init();
#endif
#if BLE_SP_SERVER
    app_scpps_init();
#endif
#if BLE_BATT_SERVER
    app_bass_init();
#endif
#if BLE_GL_SENSOR
    app_glps_init();
#endif
#if BLE_HID_DEVICE
    app_hogpd_init();
#endif

#if QN_DBG_PRINT
    app_uart_init();
#endif
#if QN_EACI
#if (defined(QN_TEST_CTRL_PIN))
    if(gpio_read_pin(QN_TEST_CTRL_PIN) == GPIO_HIGH)
#endif
    app_eaci_init();
#endif
#if (QN_WORK_MODE != WORK_MODE_SOC)
    app_gap_reset_req();
#endif
}

/// @} APP
