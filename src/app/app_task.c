/**
 ****************************************************************************************
 *
 * @file app_task.c
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

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if QN_DEMO_MENU
#include "app_menu.h"
#endif
#if (QN_WORK_MODE == WORK_MODE_NP)
#include "led.h"
#endif
#if QN_EACI
#include "app_eaci.h"
#endif

#if (QN_32K_RCO)
#include "syscon.h"
#endif

#if (QN_DEEP_SLEEP_EN || QN_32K_RCO)
#include "lib.h"
#include "sleep.h"
#endif

#if (QN_EACI&&(EACI_MSG_BUFFER_EN==TRUE))
#include "app_eaci_trans.h"
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (QN_DEMO_MENU || QN_EACI)
/**
 ****************************************************************************************
 * @brief Handles UART data indication.
 *
 * @param[in] msgid      APP_SYS_UART_DATA_IND
 * @param[in] param      Pointer to struct app_uart_data_ind
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_uart_data_ind_handler(ke_msg_id_t const msgid, struct app_uart_data_ind const *ind,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch (ke_state_get(dest_id))
    {
    case APP_INIT:
        break;
    case APP_IDLE:
    default:
#if QN_EACI
        app_eaci_msg_hdl((src_id >> 8) & 0xFF,
                                src_id & 0xFF,
                                ind->len,
                                ind->data);
#elif QN_DEMO_MENU
        if (ind->len != 0)
        {
            memcpy(app_env.input, ind->data, ind->len);
            app_menu_hdl();
        }
#endif
        break;
    }

#if (QN_EACI&&(EACI_MSG_BUFFER_EN==TRUE))
    if (eaci_msg_buf_used == EACI_BUF_DEEP) {
        eaci_trans_init();
    }
    if (eaci_msg_buf_used) {
        eaci_msg_buf_used--;
    }
    return (KE_MSG_NO_FREE);
#else
    return (KE_MSG_CONSUMED);
#endif
}
#endif

/**
 ****************************************************************************************
 * @brief Handles 32k rco periodic calibration.
 *
 * @param[in] msgid      APP_SYS_RCO_CAL_TIMER
 * @param[in] param      None
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if (QN_32K_RCO)
static int app_rco_cal_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    clock_32k_correction_enable(clock_32k_correction_cb);

    dev_prevent_sleep(PM_MASK_TIMER1_ACTIVE_BIT);

    ke_timer_set(APP_SYS_RCO_CAL_TIMER, TASK_APP, 100);

    return (KE_MSG_CONSUMED);
}
#endif

/**
 ****************************************************************************************
 * @brief Handles 32k XTAL wakeup precedure.
 *
 * @param[in] msgid      APP_SYS_32K_XTAL_WAKEUP_TIMER
 * @param[in] param      None
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if (QN_DEEP_SLEEP_EN && !QN_32K_RCO)
static int app_32k_xtal_wakeup_timer(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // disable schmitt trigger in 32.768KHz buffer
    syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32SMT_EN, MASK_DISABLE);
    syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32ICTRL, 16);
    
    // switch 32k RCO to 32k XTAL
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, MASK_DISABLE);
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RCO, MASK_ENABLE);

    // allow ble sleep
    enable_ble_sleep(true);

    return (KE_MSG_CONSUMED);
}
#endif

/* default State handlers definition. */
const struct ke_msg_handler app_default_state[] =
{
#ifdef QN_BLE_TIME_SUPPORT
    {APP_SYS_TIME_CHECK_TIMER,              (ke_msg_func_t) app_sys_time_check_timer_handler},
#endif

#if (QN_EACI || QN_DEMO_MENU)
    {APP_SYS_UART_DATA_IND,                 (ke_msg_func_t) app_uart_data_ind_handler},
#endif

#if (QN_32K_RCO)
    {APP_SYS_RCO_CAL_TIMER,                 (ke_msg_func_t) app_rco_cal_timer_handler},
#endif

#if (QN_DEEP_SLEEP_EN && !QN_32K_RCO)
    {APP_SYS_32K_XTAL_WAKEUP_TIMER,         (ke_msg_func_t) app_32k_xtal_wakeup_timer},
#endif

#if (!QN_WORK_MODE && !QN_EAPI)
#if (BLE_PERIPHERAL || BLE_BROADCASTER || BLE_OBSERVER)
    {APP_SYS_LED_1_TIMER,                   (ke_msg_func_t) app_led_timer_handler},
		{APP_BEACON_CHG_CTX_TIMER,              (ke_msg_func_t) app_beacon_chg_ctx_timer_handler},
#if (!QN_EACI)
    {APP_SYS_LED_2_TIMER,                   (ke_msg_func_t) app_led_timer_handler},
#if (BLE_PERIPHERAL)
    {APP_ADV_INTV_UPDATE_TIMER,             (ke_msg_func_t) app_gap_adv_intv_update_timer_handler},
#endif
    {APP_SYS_BUTTON_1_TIMER,                (ke_msg_func_t) app_button_timer_handler},
    {APP_SYS_BUTTON_2_TIMER,                (ke_msg_func_t) app_button_timer_handler},
#endif
#endif
#endif

    {GAP_RESET_REQ_CMP_EVT,                 (ke_msg_func_t) app_gap_reset_req_cmp_handler},
    {GAP_SET_SEC_REQ_CMP_EVT,               (ke_msg_func_t) app_gap_set_sec_req_cmp_evt_handler},
    {GAP_READ_VER_REQ_CMP_EVT,              (ke_msg_func_t) app_gap_read_ver_req_cmp_evt_handler},
    {GAP_READ_BDADDR_REQ_CMP_EVT,           (ke_msg_func_t) app_gap_read_bdaddr_req_cmp_evt_handler},
    {GAP_READY_EVT,                         (ke_msg_func_t) app_gap_ready_evt_handler},
    {GAP_SET_DEVNAME_REQ_CMP_EVT,           (ke_msg_func_t) app_gap_set_devname_req_cmp_evt_handler},
#if (BLE_PERIPHERAL)
    {GAP_ADV_REQ_CMP_EVT,                   (ke_msg_func_t) app_gap_adv_req_cmp_evt_handler},
    {GAP_PARAM_UPDATE_RESP,                 (ke_msg_func_t) app_gap_param_update_resp_handler},
    {GAP_SET_PRIVACY_REQ_CMP_EVT,           (ke_msg_func_t) app_gap_set_privacy_req_cmp_handler},
#endif
#if (BLE_CENTRAL || BLE_OBSERVER)
    {GAP_DEV_INQ_REQ_CMP_EVT,               (ke_msg_func_t) app_gap_dev_inq_cmp_handler},
#endif
#if (BLE_OBSERVER)
    {GAP_ADV_REPORT_EVT,                    (ke_msg_func_t) app_gap_dev_scan_result_handler},
#endif
#if (BLE_CENTRAL)
    {GAP_DEV_INQ_RESULT_EVT,                (ke_msg_func_t) app_gap_dev_inq_result_handler},
    {GAP_SCAN_REQ_CMP_EVT,                  (ke_msg_func_t) app_gap_scan_req_cmp_evt_handler},
    {GAP_CANCEL_CONN_REQ_CMP_EVT,           (ke_msg_func_t) app_gap_cancel_conn_req_cmp_evt_handler},
    {GAP_NAME_REQ_CMP_EVT,                  (ke_msg_func_t) app_gap_name_req_cmp_handler},
    {GAP_PARAM_UPDATE_REQ_IND,              (ke_msg_func_t) app_gap_param_update_req_ind_handler},
    {GAP_SET_RECON_ADDR_REQ_CMP_EVT,        (ke_msg_func_t) app_gap_set_recon_addr_req_cmp_handler},
    {GAP_SET_PH_PRIVACY_REQ_CMP_EVT,        (ke_msg_func_t) app_gap_set_ph_privacy_req_cmp_handler},
#endif
    {GAP_CHANGE_PARAM_REQ_CMP_EVT,          (ke_msg_func_t) app_gap_change_param_req_cmp_handler},
    {GAP_SET_MODE_REQ_CMP_EVT,              (ke_msg_func_t) app_gap_set_mode_req_cmp_evt_handler},
#if (!BLE_BROADCASTER && !BLE_OBSERVER)
    {GAP_LE_CREATE_CONN_REQ_CMP_EVT,        (ke_msg_func_t) app_gap_le_create_conn_req_cmp_evt_handler},
    {GAP_DISCON_CMP_EVT,                    (ke_msg_func_t) app_gap_discon_cmp_evt_handler},
#endif
#if (!BLE_BROADCASTER && !BLE_OBSERVER)
    {GAP_BOND_REQ_IND,                      (ke_msg_func_t) app_gap_bond_req_ind_handler},
#endif
#if (QN_SECURITY_ON)
    {GAP_BOND_REQ_CMP_EVT,                  (ke_msg_func_t) app_gap_bond_req_cmp_ind_handler},
#endif
#if (QN_WL_SUPPORT)
    {GAP_LE_RD_WLST_SIZE_CMD_CMP_EVT,       (ke_msg_func_t) app_gap_le_rd_wlst_size_cmd_cmp_handler},
    {GAP_LE_ADD_DEV_TO_WLST_REQ_CMP_EVT,    (ke_msg_func_t) app_gap_le_add_dev_to_wlst_req_cmp_handler},
    {GAP_LE_RMV_DEV_FRM_WLST_REQ_CMP_EVT,   (ke_msg_func_t) app_gap_le_rmv_dev_from_wlst_req_cmp_handler},
#endif
#if (QN_LINK_INFO)
    {GAP_LE_RD_REMOTE_FEAT_REQ_CMP_EVT,     (ke_msg_func_t) app_gap_le_rd_remote_feat_req_cmp_handler},
    {GAP_RD_REM_VER_INFO_CMP_EVT,           (ke_msg_func_t) app_gap_rd_remote_ver_info_cmp_handler},
    {GAP_CHANNEL_MAP_CMP_EVT,               (ke_msg_func_t) app_gap_channel_map_cmp_handler},
    {GAP_READ_RSSI_REQ_CMP_EVT,             (ke_msg_func_t) app_gap_read_rssi_req_cmp_handler},
#endif
    {GAP_SET_RANDOM_ADDR_REQ_CMP_EVT,       (ke_msg_func_t) app_gap_set_random_add_req_cmp_handler},

    {GATT_RESOURCE_ACCESS_REQ,              (ke_msg_func_t) app_gatt_resource_access_req_handler},
#if QN_SVC_DISC_USED
    {GATT_DISC_SVC_ALL_CMP_EVT,             (ke_msg_func_t) app_gatt_disc_svc_all_cmp_evt_handler},
    {GATT_DISC_SVC_BY_UUID_CMP_EVT,         (ke_msg_func_t) app_gatt_disc_svc_by_uuid_cmp_evt_handler},
    {GATT_DISC_SVC_INCL_CMP_EVT,            (ke_msg_func_t) app_gatt_disc_svc_incl_cmp_evt_handler},
    {GATT_DISC_SVC_ALL_128_CMP_EVT,         (ke_msg_func_t) app_gatt_disc_svc_all_128_cmp_evt_handler},
    {GATT_DISC_CHAR_ALL_CMP_EVT,            (ke_msg_func_t) app_gatt_disc_char_all_cmp_evt_handler},
    {GATT_DISC_CHAR_BY_UUID_CMP_EVT,        (ke_msg_func_t) app_gatt_disc_char_by_uuid_cmp_evt_handler},
    {GATT_DISC_CHAR_ALL_128_CMP_EVT,        (ke_msg_func_t) app_gatt_disc_char_all_128_cmp_evt_handler},
    {GATT_DISC_CHAR_BY_UUID_128_CMP_EVT,    (ke_msg_func_t) app_gatt_disc_char_by_uuid_128_cmp_evt_handler},
    {GATT_DISC_CHAR_DESC_CMP_EVT,           (ke_msg_func_t) app_gatt_disc_char_desc_cmp_evt_handler},
    {GATT_DISC_CHAR_DESC_128_CMP_EVT,       (ke_msg_func_t) app_gatt_disc_char_desc_128_cmp_evt_handler},
    {GATT_READ_CHAR_RESP,                   (ke_msg_func_t) app_gatt_read_char_resp_handler},
    {GATT_READ_CHAR_MULT_RESP,              (ke_msg_func_t) app_gatt_read_char_mult_resp_handler},
    {GATT_WRITE_CHAR_RESP,                  (ke_msg_func_t) app_gatt_write_char_resp_handler},
    {GATT_WRITE_CHAR_RELIABLE_RESP,         (ke_msg_func_t) app_gatt_write_char_reliable_resp_handler},
    {GATT_CANCEL_WRITE_CHAR_RESP,           (ke_msg_func_t) app_gatt_cancel_write_char_resp_handler},
    {GATT_NOTIFY_CMP_EVT,                   (ke_msg_func_t) app_gatt_notify_cmp_evt_handler},
    {GATT_HANDLE_VALUE_NOTIF,               (ke_msg_func_t) app_gatt_handle_value_notif_handler},
    {GATT_HANDLE_VALUE_IND,                 (ke_msg_func_t) app_gatt_handle_value_ind_handler},
    {GATT_HANDLE_VALUE_CFM,                 (ke_msg_func_t) app_gatt_handle_value_cfm_handler},
    {GATT_DISC_CMP_EVT,                     (ke_msg_func_t) app_gatt_disc_cmp_evt_handler},
    {GATT_CMP_EVT,                          (ke_msg_func_t) app_gatt_cmp_evt_handler},
#endif

#if (!BLE_BROADCASTER && !BLE_OBSERVER)
    {SMPM_SET_KEY_CFM,                      (ke_msg_func_t) app_smpm_set_key_cfm_handler},
#endif
#if (QN_SECURITY_ON)
    {SMPC_KEY_IND,                          (ke_msg_func_t) app_smpc_key_ind_handler},
    {SMPC_TK_REQ_IND,                       (ke_msg_func_t) app_smpc_tk_req_ind_handler},    
    {SMPC_LTK_REQ_IND,                      (ke_msg_func_t) app_smpc_ltk_req_ind_handler},
    {SMPC_IRK_REQ_IND,                      (ke_msg_func_t) app_smpc_irk_req_ind_handler},
    {SMPC_CHK_BD_ADDR_REQ_IND,              (ke_msg_func_t) app_smpc_chk_bd_addr_req_ind_handler},
    {SMPC_SEC_STARTED_IND,                  (ke_msg_func_t) app_smpc_sec_started_ind_handler},
    {SMPC_TIMEOUT_EVT,                      (ke_msg_func_t) app_smpc_timeout_evt_handler},
#endif
#if (QN_CSRK_SUPPORT)
    {SMPC_CSRK_REQ_IND,                     (ke_msg_func_t) app_smpc_csrk_req_ind_handler},
#endif

#if BLE_HT_COLLECTOR
    {HTPC_ENABLE_CFM,                   	(ke_msg_func_t) app_htpc_enable_cfm_handler},
    {HTPC_ERROR_IND,                    	(ke_msg_func_t) app_htpc_error_ind_handler},
    {HTPC_RD_CHAR_RSP,                  	(ke_msg_func_t) app_htpc_rd_char_rsp_handler},
    {HTPC_WR_CHAR_RSP,                  	(ke_msg_func_t) app_htpc_wr_char_rsp_handler},
    {HTPC_TEMP_IND,                     	(ke_msg_func_t) app_htpc_temp_ind_handler},
    {HTPC_MEAS_INTV_IND,                	(ke_msg_func_t) app_htpc_meas_intv_ind_handler},
    {HTPC_DISABLE_IND,                      (ke_msg_func_t) app_htpc_disable_ind_handler},
#endif

#if BLE_HT_THERMOM
    {HTPT_CREATE_DB_CFM,                    (ke_msg_func_t) app_htpt_create_db_cfm_handler},
    {HTPT_DISABLE_IND,                      (ke_msg_func_t) app_htpt_disable_ind_handler},
    {HTPT_ERROR_IND,                    	(ke_msg_func_t) app_htpt_error_ind_handler},
    {HTPT_TEMP_SEND_CFM,                	(ke_msg_func_t) app_htpt_temp_send_cfm_handler}, 
    {HTPT_MEAS_INTV_CHG_IND,            	(ke_msg_func_t) app_htpt_meas_intv_chg_ind_handler},
    {HTPT_CFG_INDNTF_IND,               	(ke_msg_func_t) app_htpt_cfg_indntf_ind_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_HTPT_PERIOD_MEAS_TIMER,            (ke_msg_func_t) app_htpt_period_meas_timer_handler},
    {APP_HTPT_IDLE_CONNECTION_TIMEOUT_TIMER,(ke_msg_func_t) app_htpt_idle_connection_timeout_timer_handler},
    #endif
#endif

#if BLE_BP_COLLECTOR
    {BLPC_ENABLE_CFM,                   	(ke_msg_func_t) app_blpc_enable_cfm_handler},
    {BLPC_ERROR_IND,                    	(ke_msg_func_t) app_blpc_error_ind_handler},
    {BLPC_RD_CHAR_RSP,                  	(ke_msg_func_t) app_blpc_rd_char_rsp_handler},
    {BLPC_WR_CHAR_RSP,                  	(ke_msg_func_t) app_blpc_wr_char_rsp_handler},
    {BLPC_BP_MEAS_IND,                  	(ke_msg_func_t) app_blpc_bp_meas_ind_handler},
    {BLPC_DISABLE_IND,                      (ke_msg_func_t) app_blpc_disable_ind_handler},
#endif

#if BLE_BP_SENSOR
    {BLPS_DISABLE_IND,                      (ke_msg_func_t) app_blps_disable_ind_handler},
    {BLPS_ERROR_IND,                        (ke_msg_func_t) app_blps_error_ind_handler},
    {BLPS_MEAS_SEND_CFM,                    (ke_msg_func_t) app_blps_meas_send_cfm_handler},
    {BLPS_CFG_INDNTF_IND,                   (ke_msg_func_t) app_blps_cfg_indntf_ind_handler},
    {BLPS_CREATE_DB_CFM,                    (ke_msg_func_t) app_blps_create_db_cfm_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_BLPS_PRESSURE_TIMER,               (ke_msg_func_t) app_blps_blood_send_handler},
    #endif
#endif

#if BLE_HR_COLLECTOR
    {HRPC_ENABLE_CFM,                   	(ke_msg_func_t) app_hrpc_enable_cfm_handler},
    {HRPC_ERROR_IND,                    	(ke_msg_func_t) app_hrpc_error_ind_handler},
    {HRPC_RD_CHAR_RSP,                  	(ke_msg_func_t) app_hrpc_rd_char_rsp_handler},
    {HRPC_WR_CHAR_RSP,                  	(ke_msg_func_t) app_hrpc_wr_char_rsp_handler},
    {HRPC_HR_MEAS_IND,                  	(ke_msg_func_t) app_hrpc_meas_ind_handler},
    {HRPC_DISABLE_IND,                      (ke_msg_func_t) app_hrpc_disable_ind_handler},
#endif

#if BLE_HR_SENSOR
    {HRPS_DISABLE_IND,                      (ke_msg_func_t) app_hrps_disable_ind_handler},
    {HRPS_ERROR_IND,                        (ke_msg_func_t) app_hrps_error_ind_handler},
    {HRPS_MEAS_SEND_CFM,                    (ke_msg_func_t) app_hrps_means_send_cfm_handler},
    {HRPS_CFG_INDNTF_IND,                   (ke_msg_func_t) app_hrps_cfg_indntf_ind_handler},
    {HRPS_ENERGY_EXP_RESET_IND,             (ke_msg_func_t) app_hrps_energy_exp_reset_ind_handler},
    {HRPS_CREATE_DB_CFM,                    (ke_msg_func_t) app_hrps_create_db_cfm_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_HRPS_TIMER,                        (ke_msg_func_t) app_hrps_timer_handler},
    #endif
#endif

#if BLE_GL_COLLECTOR
    {GLPC_ENABLE_CFM,                   	(ke_msg_func_t) app_glpc_enable_cfm_handler},
    {GLPC_REGISTER_CFM,                 	(ke_msg_func_t) app_glpc_register_cfm_handler},
    {GLPC_READ_FEATURES_RSP,            	(ke_msg_func_t) app_glpc_read_features_rsp_handler},
    {GLPC_RACP_RSP,                     	(ke_msg_func_t) app_glpc_racp_response_handler},
    {GLPC_MEAS_IND,                     	(ke_msg_func_t) app_glpc_meas_ind_handler},
    {GLPC_MEAS_CTX_IND,                 	(ke_msg_func_t) app_glpc_meas_ctx_ind_handler},
    {GLPC_DISABLE_IND,                      (ke_msg_func_t) app_glpc_disable_ind_handler},
#endif

#if BLE_GL_SENSOR
    {GLPS_CREATE_DB_CFM,                    (ke_msg_func_t) app_glps_create_db_cfm_handler},
    {GLPS_ENABLE_CFM,                       (ke_msg_func_t) app_glps_enable_cfm_handler},
    {GLPS_DISABLE_IND,                      (ke_msg_func_t) app_glps_disable_ind_handler},
    {GLPS_CFG_INDNTF_IND,                   (ke_msg_func_t) app_glps_cfg_indntf_ind_handler},
    {GLPS_RACP_REQ_IND,                     (ke_msg_func_t) app_glps_racp_req_ind_handler},
    {GLPS_REQ_CMP_EVT,                      (ke_msg_func_t) app_glps_req_cmp_evt_handler},
    {GLPS_ERROR_IND,                        (ke_msg_func_t) app_glps_error_ind_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_GLPS_MEAS_SEND_TIMER,              (ke_msg_func_t) app_glps_meas_send_timer_handler},
    #endif
#endif

#if BLE_FINDME_LOCATOR
    {FINDL_ENABLE_CFM,                  	(ke_msg_func_t) app_findl_enable_cfm_handler},
    {FINDL_ERROR_IND,                   	(ke_msg_func_t) app_findl_error_ind_handler},
    {FINDL_DISABLE_IND,                     (ke_msg_func_t) app_findl_disable_ind_handler},
#endif

#if BLE_FINDME_TARGET
    {FINDT_ALERT_IND,                       (ke_msg_func_t) app_findt_alert_ind_handler},
    {FINDT_CREATE_DB_CFM,                   (ke_msg_func_t) app_findt_create_db_cfm_handler},
    {FINDT_DISABLE_IND,                     (ke_msg_func_t) app_findt_disable_ind_handler},
    {FINDT_ERROR_IND,                       (ke_msg_func_t) app_findt_error_ind_handler},
#endif

#if BLE_PROX_MONITOR
    {PROXM_ENABLE_CFM,                  	(ke_msg_func_t) app_proxm_enable_cfm_handler},
    {PROXM_RD_CHAR_RSP,                 	(ke_msg_func_t) app_proxm_rd_char_rsp_handler},
    {PROXM_WR_CHAR_RSP,                 	(ke_msg_func_t) app_proxm_wr_char_rsp_handler},
    {PROXM_DISABLE_IND,                     (ke_msg_func_t) app_proxm_disable_ind_handler},
#endif

#if BLE_PROX_REPORTER
    {PROXR_DISABLE_IND,                     (ke_msg_func_t) app_proxr_disable_ind_handler},
    {PROXR_ALERT_IND,                       (ke_msg_func_t) app_proxr_alert_ind_handler},
    {PROXR_CREATE_DB_CFM,                   (ke_msg_func_t) app_proxr_create_db_cfm_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_PROXR_ALERT_STOP_TIMER,            (ke_msg_func_t) app_proxr_alert_to_handler},
    #endif
#endif

#if BLE_TIP_CLIENT
    {TIPC_ENABLE_CFM,                   	(ke_msg_func_t) app_tipc_enable_cfm_handler},
    {TIPC_ERROR_IND,                    	(ke_msg_func_t) app_tipc_error_ind_handler},
    {TIPC_WR_CHAR_RSP,                  	(ke_msg_func_t) app_tipc_wr_char_rsp_handler},
    {TIPC_CT_IND,                       	(ke_msg_func_t) app_tipc_ct_ind_handler},
    {TIPC_CT_NTF_CFG_RD_RSP,            	(ke_msg_func_t) app_tipc_ct_ntf_cfg_rd_rsp_handler},
    {TIPC_LTI_RD_RSP,                   	(ke_msg_func_t) app_tipc_lti_rd_rsp_handler},
    {TIPC_RTI_RD_RSP,                   	(ke_msg_func_t) app_tipc_rti_rd_rsp_handler},
    {TIPC_TDST_RD_RSP,                  	(ke_msg_func_t) app_tipc_tdst_rd_rsp_handler},
    {TIPC_TUS_RD_RSP,                   	(ke_msg_func_t) app_tipc_tus_rd_rsp_handler},
    {TIPC_DISABLE_IND,                      (ke_msg_func_t) app_tipc_disable_ind_handler},
#endif

#if BLE_TIP_SERVER
    {TIPS_DISABLE_IND,                  	(ke_msg_func_t) app_tips_disable_ind_handler},
    {TIPS_ERROR_IND,                    	(ke_msg_func_t) app_tips_error_ind_handler},
    {TIPS_CURRENT_TIME_CCC_IND,         	(ke_msg_func_t) app_tips_current_time_ccc_ind_handler},
    {TIPS_TIME_UPD_CTNL_PT_IND,         	(ke_msg_func_t) app_tips_time_upd_ctnl_pt_ind_handler},
    {TIPS_CREATE_DB_CFM,                	(ke_msg_func_t) app_tips_create_db_cfm_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_TIPS_CURRENT_TIME_TIMER,       	(ke_msg_func_t) app_tips_current_time_timer_handler},
    #endif
#endif

#if BLE_HID_BOOT_HOST
    {HOGPBH_ENABLE_CFM,                 	(ke_msg_func_t) app_hogpbh_enable_cfm_handler},
    {HOGPBH_WR_CHAR_RSP,                	(ke_msg_func_t) app_hogpbh_wr_char_rsp_handler},
    {HOGPBH_CFG_NTF_RD_RSP,             	(ke_msg_func_t) app_hogpbh_cfg_ntf_rd_rsp_handler},
    {HOGPBH_PROTO_MODE_RD_RSP,          	(ke_msg_func_t) app_hogpbh_proto_mode_rd_rsp_handler},
    {HOGPBH_BOOT_REPORT_IND,      	        (ke_msg_func_t) app_hogpbh_boot_report_ind_handler},
    {HOGPBH_RD_CHAR_ERR_RSP,            	(ke_msg_func_t) app_hogpbh_char_req_rsp_handler},
    {HOGPBH_DISABLE_IND,                    (ke_msg_func_t) app_hogpbh_disable_ind_handler},
#endif

#if BLE_HID_REPORT_HOST
    {HOGPRH_ENABLE_CFM,                 	(ke_msg_func_t) app_hogprh_enable_cfm_handler},
    {HOGPRH_WR_CHAR_RSP,                	(ke_msg_func_t) app_hogprh_wr_char_rsp_handler},
    {HOGPRH_PROTO_MODE_RD_RSP,          	(ke_msg_func_t) app_hogprh_proto_mode_rd_rsp_handler},
    {HOGPRH_REPORT_MAP_RD_RSP,          	(ke_msg_func_t) app_hogprh_report_map_rd_rsp_handler},
    {HOGPRH_REPORT_MAP_EXT_REP_REF_RD_RSP,	(ke_msg_func_t) app_hogprh_report_map_ext_rep_ref_rd_rsp_handler},
    {HOGPRH_REPORT_REP_REF_RD_RSP,      	(ke_msg_func_t) app_hogprh_report_ref_rd_rsp_handler},
    {HOGPRH_HID_INFO_RD_RSP,            	(ke_msg_func_t) app_hogprh_hid_info_rd_rsp_handler},
    {HOGPRH_NTF_CFG_RD_RSP,             	(ke_msg_func_t) app_hogprh_cfg_ntf_rd_rsp_handler},
    {HOGPRH_REPORT_IND,                 	(ke_msg_func_t) app_hogprh_report_ind_handler},
    {HOGPRH_RD_CHAR_ERR_RSP,            	(ke_msg_func_t) app_hogprh_char_req_rsp_handler},
    {HOGPRH_DISABLE_IND,                    (ke_msg_func_t) app_hogprh_disable_ind_handler},
#endif

#if BLE_HID_DEVICE
    {HOGPD_CREATE_DB_CFM,               	(ke_msg_func_t) app_hogpd_create_db_cfm_handler},
    {HOGPD_DISABLE_IND,                 	(ke_msg_func_t) app_hogpd_disable_ind_handler},
    {HOGPD_ERROR_IND,                   	(ke_msg_func_t) app_hogpd_error_ind_handler},
    {HOGPD_NTF_CFG_IND,                 	(ke_msg_func_t) app_hogpd_ntf_cfg_ind_handler},
    {HOGPD_PROTO_MODE_IND,              	(ke_msg_func_t) app_hogpd_proto_mode_ind_handler},
    {HOGPD_REPORT_IND,                  	(ke_msg_func_t) app_hogpd_report_ind_handler},
    {HOGPD_BOOT_REPORT_IND,           	    (ke_msg_func_t) app_hogpd_boot_report_ind_handler},
    {HOGPD_CTNL_PT_IND,                 	(ke_msg_func_t) app_hogpd_ctnl_pt_ind_handler},
    {HOGPD_NTF_SENT_CFM,                	(ke_msg_func_t) app_hogpd_ntf_sent_cfm_handler},
    #if (!QN_EACI && !QN_EAPI)
    {APP_HOGPD_BOOT_KB_IN_REPORT_TIMER,     (ke_msg_func_t) app_hogpd_boot_kb_in_report_timer_handler},
    {APP_HOGPD_BOOT_MOUSE_IN_REPORT_TIMER,  (ke_msg_func_t) app_hogpd_boot_mouse_in_report_timer_handler},
    {APP_HOGPD_REPORT_TIMER,                (ke_msg_func_t) app_hogpd_report_timer_handler},
    #endif
#endif

#if BLE_SP_CLIENT
    {SCPPC_ENABLE_CFM,                  	(ke_msg_func_t) app_scppc_enable_cfm_handler},
    {SCPPC_ERROR_IND,                   	(ke_msg_func_t) app_scppc_error_ind_handler},
    {SCPPC_WR_CHAR_RSP,                 	(ke_msg_func_t) app_scppc_wr_char_rsp_handler},
    {SCPPC_SCAN_REFRESH_NTF_CFG_RD_RSP, 	(ke_msg_func_t) app_scppc_scan_refresh_ntf_cfg_rd_rsp_handler},
    {SCPPC_DISABLE_IND,                     (ke_msg_func_t) app_scppc_disable_ind_handler},
#endif

#if BLE_SP_SERVER
    {SCPPS_CREATE_DB_CFM,                   (ke_msg_func_t) app_scpps_create_db_cfm_handler},
    {SCPPS_DISABLE_IND,                     (ke_msg_func_t) app_scpps_disable_ind_handler},
    {SCPPS_ERROR_IND,                       (ke_msg_func_t) app_scpps_error_ind_handler},
    {SCPPS_SCAN_INTV_WD_IND,                (ke_msg_func_t) app_scpps_scan_intv_wd_ind_handler},
    {SCPPS_SCAN_REFRESH_NTF_CFG_IND,        (ke_msg_func_t) app_scpps_scan_refresh_ntf_cfg_ind_handler},
    {SCPPS_SCAN_REFRESH_SEND_CFM,           (ke_msg_func_t) app_scpps_scan_refresh_send_cfm_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_SCPPS_SCAN_REFRESH_TIMER,          (ke_msg_func_t) app_scpps_scan_refresh_timer_handler},
    #endif
#endif

#if BLE_DIS_CLIENT
    {DISC_ENABLE_CFM,                   	(ke_msg_func_t) app_disc_enable_cfm_handler},
    {DISC_RD_CHAR_RSP,                  	(ke_msg_func_t) app_disc_rd_char_rsp_handler},
    {DISC_DISABLE_IND,                      (ke_msg_func_t) app_disc_disable_ind_handler},
#endif

#if BLE_DIS_SERVER
    {DISS_CREATE_DB_CFM,                    (ke_msg_func_t) app_diss_create_db_cfm_handler}, 
    {DISS_ERROR_IND,                        (ke_msg_func_t) app_diss_error_ind_handler},
#endif

#if BLE_BATT_CLIENT
    {BASC_ENABLE_CFM,                   	(ke_msg_func_t) app_basc_enable_cfm_handler},
    {BASC_ERROR_IND,                    	(ke_msg_func_t) app_basc_error_ind_handler},
    {BASC_WR_CHAR_RSP,                  	(ke_msg_func_t) app_basc_wr_char_rsp_handler},
    {BASC_BATT_LEVEL_NTF_CFG_RD_RSP,    	(ke_msg_func_t) app_basc_batt_level_ntf_cfg_rd_rsp_handler},
    {BASC_BATT_LEVEL_PRES_FORMAT_RD_RSP,	(ke_msg_func_t) app_basc_batt_level_pres_format_rd_rsp_handler},
    {BASC_BATT_LEVEL_IND,               	(ke_msg_func_t) app_basc_batt_level_ind_handler},
    {BASC_DISABLE_IND,                      (ke_msg_func_t) app_basc_disable_ind_handler},
#endif

#if BLE_BATT_SERVER
    {BASS_CREATE_DB_CFM,                    (ke_msg_func_t) app_bass_create_db_cfm_handler},
    {BASS_DISABLE_IND,                      (ke_msg_func_t) app_bass_disable_ind_handler},
    {BASS_ERROR_IND,                        (ke_msg_func_t) app_bass_error_ind_handler},
    {BASS_BATT_LEVEL_UPD_CFM,               (ke_msg_func_t) app_bass_batt_level_upd_cfm_handler},
    {BASS_BATT_LEVEL_NTF_CFG_IND,           (ke_msg_func_t) app_bass_batt_level_ntf_cfg_ind_handler},
    #if (!QN_WORK_MODE && !QN_EACI && !QN_EAPI)
    {APP_BASS_BATT_LEVEL_TIMER,             (ke_msg_func_t) app_bass_batt_level_timer_handler},
    #endif
#endif
    
#if BLE_CSC_COLLECTOR
    {CSCPC_CSCS_CONTENT_IND,                (ke_msg_func_t) app_cscpc_cscs_content_ind_handler},
    {CSCPC_VALUE_IND,                       (ke_msg_func_t) app_cscpc_value_ind_handler},
    {CSCPC_CMP_EVT,                         (ke_msg_func_t) app_cscpc_cmp_evt_handler},
    {CSCPC_DISABLE_IND,                     (ke_msg_func_t) app_cscpc_disable_ind_handler},
#endif
    
#if BLE_CSC_SENSOR
    {CSCPS_DISABLE_IND,                     (ke_msg_func_t) app_cscps_disable_ind_handler},
    {CSCPS_SC_CTNL_PT_REQ_IND,              (ke_msg_func_t) app_cscps_ctnl_pt_req_ind_handler},
    {CSCPS_NTF_IND_CFG_IND,                 (ke_msg_func_t) app_cscps_ntf_ind_cfg_ind_handler},
    {CSCPS_CMP_EVT,                         (ke_msg_func_t) app_cscps_cmp_evt_handler},
#if (!QN_EACI && !QN_EAPI)
    {APP_CSCPS_MEAS_SEND_TIMER,             (ke_msg_func_t) app_cscps_meas_send_timer_handler},
#endif
#endif

#if BLE_RSC_COLLECTOR
    {RSCPC_RSCS_CONTENT_IND,                (ke_msg_func_t) app_rscpc_rscs_content_ind_handler},
    {RSCPC_VALUE_IND,                       (ke_msg_func_t) app_rscpc_value_ind_handler},
    {RSCPC_CMP_EVT,                         (ke_msg_func_t) app_rscpc_cmp_evt_handler},
    {RSCPC_DISABLE_IND,                     (ke_msg_func_t) app_rscpc_disable_ind_handler},
#endif

#if BLE_RSC_SENSOR
    {RSCPS_DISABLE_IND,                     (ke_msg_func_t) app_rscps_disable_ind_handler},
    {RSCPS_SC_CTNL_PT_REQ_IND,              (ke_msg_func_t) app_rscps_ctnl_pt_req_ind_handler},
    {RSCPS_NTF_IND_CFG_IND,                 (ke_msg_func_t) app_rscps_ntf_ind_cfg_ind_handler},
    {RSCPS_CMP_EVT,                         (ke_msg_func_t) app_rscps_cmp_evt_handler},
#if (!QN_EACI && !QN_EAPI)
    {APP_RSCPS_MEAS_SEND_TIMER,             (ke_msg_func_t) app_rscps_meas_send_timer_handler},
#endif
#endif

#if BLE_PAS_CLIENT
    {PASPC_DISABLE_IND,                     (ke_msg_func_t) app_paspc_disable_ind_handler},
    {PASPC_VALUE_IND,                       (ke_msg_func_t) app_paspc_val_ind_handler},
    {PASPC_PASS_CONTENT_IND,                (ke_msg_func_t) app_paspc_pass_content_handler},
    {PASPC_CMP_EVT,                         (ke_msg_func_t) app_paspc_cmp_evt_handler},
#endif

#if BLE_PAS_SERVER
    {PASPS_DISABLE_IND,                     (ke_msg_func_t) app_pasps_disable_ind_handler},
    {PASPS_WRITTEN_CHAR_VAL_IND,            (ke_msg_func_t) app_pasps_written_char_val_ind_handler},
    {PASPS_CMP_EVT,                         (ke_msg_func_t) app_pasps_cmp_evt_handler},
#if (!QN_EACI && !QN_EAPI)
    {APP_PASPS_UPDATE_ALERT_STATUS_TIMER,   (ke_msg_func_t) app_pasps_update_alert_timer_handler},
    {APP_PASPS_UPDATE_RINGER_TIMER,         (ke_msg_func_t) app_pasps_update_ringer_timer_handler},
#endif
#endif

#if BLE_AN_CLIENT
    {ANPC_DISABLE_IND,                      (ke_msg_func_t) app_anpc_disable_ind_handler},
    {ANPC_VALUE_IND,                        (ke_msg_func_t) app_anpc_value_ind_handler},
    {ANPC_ANS_CONTENT_IND,                  (ke_msg_func_t) app_anpc_content_ind_handler},
    {ANPC_CMP_EVT,                          (ke_msg_func_t) app_anpc_cmp_evt_handler},
#endif

#if BLE_AN_SERVER
    {ANPS_DISABLE_IND,                      (ke_msg_func_t) app_anps_disable_ind_handler},
    {ANPS_NTF_IMMEDIATE_REQ_IND,            (ke_msg_func_t) app_anps_ntf_immediate_req_ind_handler},
    {ANPS_NTF_STATUS_UPDATE_IND,            (ke_msg_func_t) app_anps_ntf_status_update_ind_handler},
    {ANPS_CMP_EVT,                          (ke_msg_func_t) app_anps_cmp_evt_handler},
#endif

#if BLE_QPP_CLIENT
    {QPPC_ENABLE_CFM,                   	(ke_msg_func_t) app_qppc_enable_cfm_handler},
    {QPPC_ERROR_IND,                    	(ke_msg_func_t) app_qppc_error_ind_handler},
    {QPPC_RD_CHAR_RSP,                  	(ke_msg_func_t) app_qppc_rd_char_rsp_handler},
    {QPPC_WR_CHAR_RSP,                  	(ke_msg_func_t) app_qppc_wr_char_rsp_handler},
    {QPPC_DATA_IND,                  	    (ke_msg_func_t) app_qppc_data_ind_handler},
    {QPPC_DISABLE_IND,                      (ke_msg_func_t) app_qppc_disable_ind_handler},
#endif

#if BLE_QPP_SERVER
    {QPPS_DISABLE_IND,                  	(ke_msg_func_t) app_qpps_disable_ind_handler},
    {QPPS_ERROR_IND,                    	(ke_msg_func_t) app_qpps_error_ind_handler},
    {QPPS_DATA_SEND_CFM,                	(ke_msg_func_t) app_qpps_data_send_cfm_handler},
    {QPPS_CFG_INDNTF_IND,               	(ke_msg_func_t) app_qpps_cfg_indntf_ind_handler},
    {QPPS_DAVA_VAL_IND,         	        (ke_msg_func_t) app_qpps_data_ind_handler},
    {QPPS_CREATE_DB_CFM,                	(ke_msg_func_t) app_qpps_create_db_cfm_handler},
#endif
    
#if BLE_OTA_SERVER 
    {OTAS_TRANSIMIT_STATUS_IND,             (ke_msg_func_t) app_otas_start_handler},
#endif    
    
#if BLE_ANCS_NC
    {ANCSC_ANCS_CONTENT_IND,                (ke_msg_func_t) app_ancsc_ancs_content_ind_handler},
    {ANCSC_NTF_SOURCE_IND,                  (ke_msg_func_t) app_ancsc_ntf_source_ind_handler},
    {ANCSC_DATA_SOURCE_IND,                 (ke_msg_func_t) app_ancsc_data_source_ind_handler},
    {ANCSC_CMP_EVT,                         (ke_msg_func_t) app_ancsc_cmp_evt_handler},
    {ANCSC_DISABLE_IND,                     (ke_msg_func_t) app_ancsc_disable_ind_handler},
    {ANCS_ENABLE_SUVPER_TIMER0,             (ke_msg_func_t) app_ancsc_enable_reset_timer_handler},
    {ANCS_ENABLE_SUVPER_TIMER1,             (ke_msg_func_t) app_ancsc_enable_reset_timer_handler},
 
#endif
};

/* Specifies the app message handler structure for every input state.*/
const struct ke_state_handler app_state_handler[APP_STATE_MAX] =
{
    [APP_INIT] = KE_STATE_HANDLER_NONE,
    [APP_IDLE] = KE_STATE_HANDLER_NONE,
    [APP_ADV ] = KE_STATE_HANDLER_NONE,
    [APP_SCAN] = KE_STATE_HANDLER_NONE,
};

/* Specifies the app message handlers that are common to all states. */
const struct ke_state_handler app_default_handler = KE_STATE_HANDLER(app_default_state);

/* Defines the place holder for the states of all the app task instances. */
ke_state_t app_state[APP_IDX_MAX];

#if !QN_WORK_MODE
// Register app task into kernel
void task_app_desc_register(void)
{
    struct ke_task_desc task_app_desc;
    
    task_app_desc.state_handler = app_state_handler;
    task_app_desc.default_handler = &app_default_handler;
    task_app_desc.state = app_state;
    task_app_desc.state_max = APP_STATE_MAX;
    task_app_desc.idx_max = APP_IDX_MAX;
    
    task_desc_register(TASK_APP, task_app_desc);
}
#endif
