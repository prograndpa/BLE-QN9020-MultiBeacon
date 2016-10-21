/**
 ****************************************************************************************
 *
 * @file app_util.c
 *
 * @brief Application Utility API
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#include "app_env.h"
#if !QN_WORK_MODE
#include "qnrf.h"
#endif
#include "lib.h"
#if (defined(CFG_EACI))
extern struct device_name_set device_name;
#endif

/**
 ****************************************************************************************
 * @brief Convert u32 value to array with Hex format
 *
 ****************************************************************************************
 */
void app_i_to_array(uint32_t num, char *array)
{
    int key_len = KEY_LEN;
    uint8_t base = 16;
    
    while (num != 0)
    {
        array[--key_len] = (uint8_t)(num % base);
        num = num / base;
        
        array[key_len] |= ((uint8_t)(num % base) << 4) & 0xF0;
        num = num / base;
    }
}

/**
 ****************************************************************************************
 * @brief Set local device role
 *
 ****************************************************************************************
 */
void app_set_role(uint8_t role)
{
    // should be GAP_CENTRAL_MST, GAP_PERIPHERAL_SLV, GAP_BROADCASTER_ADV or GAP_OBSERVER_SCA
    app_env.role = role;
}

/**
 ****************************************************************************************
 * @brief Set local device role
 *
 ****************************************************************************************
 */
uint8_t app_get_role(void)
{
    return app_env.role;
}

/**
 ****************************************************************************************
 * @brief Get Device Record Index by connection handle
 *
 ****************************************************************************************
 */
uint8_t app_get_rec_idx_by_conhdl(uint16_t conhdl)
{
    uint8_t idx;

    for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
        if (app_env.dev_rec[idx].conhdl == conhdl)
            return idx;
    }

    idx = GAP_INVALID_CONIDX;
    return idx;
}


/**
 ****************************************************************************************
 * @brief Get Connection Handle by Index 
 *
 ****************************************************************************************
 */
uint16_t app_get_conhdl_by_idx(uint8_t idx)
{
    if (app_get_link_status_by_idx(idx))
        return app_env.dev_rec[idx].conhdl;
    else
        return 0xFFFF;
}

/**
 ****************************************************************************************
 * @brief Get Connection Handle and Index by BD address
 *
 ****************************************************************************************
 */
uint16_t app_get_conhdl_by_bdaddr(struct bd_addr const *addr, uint8_t *idx)
{
    for (uint8_t i = 0; i < BLE_CONNECTION_MAX; i++)
    {
        if (co_bt_bdaddr_compare(addr, &app_env.dev_rec[i].bonded_info.peer_addr) == true)
        {
            ASSERT_ERR(app_env.dev_rec[i].free == false);
            *idx = i;

            return app_env.dev_rec[i].conhdl;
        }
    }

    *idx = GAP_INVALID_CONIDX;
    return 0xFFFF;
}

/**
 ****************************************************************************************
 * @brief Get BD Addr by connection handle
 *
 ****************************************************************************************
 */
uint8_t app_get_bd_addr_by_conhdl(uint16_t conhdl, struct bd_addr *addr)
{
    uint8_t idx = GAP_INVALID_CONIDX;

    for (uint8_t i = 0; i < BLE_CONNECTION_MAX; i++)
    {
        if (app_env.dev_rec[i].conhdl == conhdl)
        {
            if (addr)
                *addr = app_env.dev_rec[i].bonded_info.peer_addr;
            idx = i;
            break;
        }
    }

    return idx;
}        

/**
 ****************************************************************************************
 * @brief Get BD Addr by index
 *
 ****************************************************************************************
 */

bool app_get_bd_addr_by_idx(uint8_t idx, struct bd_addr *addr)
{
    bool r = false;
    
    if (addr && app_get_link_status_by_idx(idx))
    {
        *addr = app_env.dev_rec[idx].bonded_info.peer_addr;
        r = true;
    }
    
    return r;
}        


/**
 ****************************************************************************************
 * @brief Get link security status by device address
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
uint8_t app_get_lk_sec_status(struct bd_addr const *addr)
{
    uint8_t idx;

    if ((idx = app_find_bonded_dev(addr)) != GAP_INVALID_CONIDX)
    {
        if (app_env.bonded_info[idx].sec_prop == SMP_KSEC_UNAUTH_NO_MITM)
            return GAP_LK_UNAUTHENTICATED;
        else if (app_env.bonded_info[idx].sec_prop == SMP_KSEC_AUTH_MITM)
            return GAP_LK_AUTHENTICATED;
    }

    return GAP_LK_SEC_NONE;
}
#endif

/**
 ****************************************************************************************
 * @brief Get Connection status by Idx
 *
 ****************************************************************************************
 */
bool app_get_link_status_by_idx(uint8_t idx)
{
    return (bool)(app_env.dev_rec[idx].free == false);
}

/**
 ****************************************************************************************
 * @brief Set Connection status by Connection handle
 *
 ****************************************************************************************
 */
void app_set_link_status_by_conhdl(uint16_t conhdl, struct gap_link_info const *conn_info, bool conn)
{
    if (conn == true)
    {
#if (BLE_CENTRAL)
        uint8_t idx = app_get_rec_idx_by_conhdl(0xFFFF);
#else
        uint8_t idx = 0;
#endif
        if (idx != GAP_INVALID_CONIDX)
        {
            app_env.dev_rec[idx].free = false;
            app_env.dev_rec[idx].conhdl = conhdl;
            app_env.dev_rec[idx].bonded_info.addr_type = conn_info->peer_addr_type;
            app_env.dev_rec[idx].bonded_info.peer_addr = conn_info->peer_addr;
            app_env.cn_count++;
        }
    }
    else
    {
        uint8_t idx = app_get_bd_addr_by_conhdl(conhdl, NULL);
        if (idx != GAP_INVALID_CONIDX)
        {
            app_env.dev_rec[idx].free = true;
            app_env.dev_rec[idx].conhdl = 0xFFFF;
            memset(&app_env.dev_rec[idx].bonded_info, 0, sizeof(struct app_bonded_info));
            app_env.cn_count--;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Get the connection number of the device record
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
uint8_t app_get_link_nb(void)
{
    return app_env.cn_count;
}
#endif

/**
 ****************************************************************************************
 * @brief Get the bonded number of the device record
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
uint8_t app_get_bond_nb(void)
{
    return app_env.bonded_count;
}
#endif

/**
 ****************************************************************************************
 * @brief Get the bonded staus of the specific device record
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
bool app_get_bond_status(uint8_t idx, uint8_t key_flag)
{
    return app_env.bonded_info[idx].sec_prop && (app_env.bonded_info[idx].peer_distribute_keys & key_flag);
}
#endif

/**
 ****************************************************************************************
 * @brief Add the bonded device
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
bool app_add_bonded_dev(void *bonded_dev)
{
    if (app_env.bonded_count < APP_MAX_BONDED_DEVICE_NUMBER)
    {
        app_env.bonded_info[app_env.bonded_count++] = *(struct app_bonded_info *)bonded_dev;
#if (QN_NVDS_WRITE)
        if (NVDS_OK == nvds_put(APP_NVDS_DB_START_TAG + app_env.bonded_count - 1, sizeof(struct app_bonded_info), (uint8_t *)bonded_dev))
        {
            uint8_t num = app_env.bonded_count;
            if (NVDS_OK == nvds_put(APP_NVDS_DB_COUNT_TAG, sizeof(num), &num))
            {
                return true;
            }
        }
        app_env.bonded_count--;
        return false;
#endif
    }
    else
    {
#if (QN_NVDS_WRITE)
        // Replace one item
        nvds_tag_len_t length = sizeof(struct app_bonded_info);

        for (uint8_t i = 1; i < (app_env.bonded_count - 1); i++)
        {
            if (NVDS_OK != nvds_put(APP_NVDS_DB_COUNT_TAG + i - 1, length, (uint8_t *)&app_env.bonded_info[i]))
            {
                return false;
            }
            app_env.bonded_info[i - 1] = app_env.bonded_info[i];
        }
        if (NVDS_OK != nvds_put(APP_NVDS_DB_START_TAG + app_env.bonded_count - 1, length, (uint8_t *)bonded_dev))
        {
            return false;
        }
        app_env.bonded_info[app_env.bonded_count - 1] = *(struct app_bonded_info *)bonded_dev;
#endif
    }
    return true;
}
#endif

/**
 ****************************************************************************************
 * @brief Find the bonded device by device address
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
uint8_t app_find_bonded_dev(struct bd_addr const *addr)
{
    int i;

    for (i = 0; i < app_env.bonded_count; i++)
    {
        if (true == co_bt_bdaddr_compare(addr, &app_env.bonded_info[i].peer_addr))
        {
            return i;
        }
    }
    return GAP_INVALID_CONIDX;
}
#endif

/**
 ****************************************************************************************
 * @brief Check Service setup FLAG and Initiate SMP IRK and CSRK
 *
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
void app_init_local_smp_key(void)
{
    // Set key here
    // IRK firstly, then CSRK
    struct smp_key new_key;
    memcpy(new_key.key, app_env.irk, KEY_LEN);
    app_smpm_set_key_req(SMP_KDIST_IDKEY, &new_key);
}
#endif

/**
 ****************************************************************************************
 * @brief Get the local service setup inital value
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
uint16_t app_get_local_service_flag(void)
{
    uint16_t srv_flag = 0;
    
#if BLE_HT_THERMOM
    srv_flag |= BLE_HT_THERMOM_BIT;
#endif
#if BLE_BP_SENSOR
    srv_flag |= BLE_BP_SENSOR_BIT;
#endif
#if BLE_HR_SENSOR
    srv_flag |= BLE_HR_SENSOR_BIT;
#endif
#if BLE_FINDME_TARGET
    srv_flag |= BLE_FINDME_TARGET_BIT;
#endif
#if BLE_TIP_SERVER
    srv_flag |= BLE_TIP_SERVER_BIT;
#endif
#if BLE_PROX_REPORTER
    srv_flag |= BLE_PROX_REPORTER_BIT;
#endif
#if BLE_SP_SERVER
    srv_flag |= BLE_SP_SERVER_BIT;
#endif
#if BLE_GL_SENSOR
    srv_flag |= BLE_GL_SENSOR_BIT;
#endif
#if BLE_HID_DEVICE
    srv_flag |= BLE_HID_DEVICE_BIT;
#endif
#if BLE_DIS_SERVER
    srv_flag |= BLE_DIS_SERVER_BIT;
#endif
#if BLE_BATT_SERVER
    srv_flag |= BLE_BATT_SERVER_BIT;
#endif
#if BLE_CSC_SENSOR
    srv_flag |= BLE_CSCP_SERVER_BIT;
#endif
#if BLE_PAS_SERVER
    srv_flag |= BLE_PASP_SERVER_BIT;
#endif
#if BLE_RSC_SENSOR
    srv_flag |= BLE_RSCP_SERVER_BIT;
#endif
#if BLE_AN_SERVER
    srv_flag |= BLE_AN_SERVER_BIT;
#endif
#if BLE_QPP_SERVER
    srv_flag |= BLE_QPPS_SERVER_BIT;
#endif

    return srv_flag;
}
#endif

/**
 ****************************************************************************************
 * @brief Clear Service setup FLAG
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
void app_clear_local_service_flag(uint16_t srv_bit)
{
    app_env.srv_flag &= ~srv_bit;
}
#endif

/**
 ****************************************************************************************
 * @brief Get the remote service UUID enable status from application device record
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
bool app_get_remote_service_status(uint8_t idx, uint16_t uuid)
{
    bool r = false;
#if QN_SVC_DISC_USED
    for (uint8_t i = 0; i < app_env.dev_rec[idx].nb_svc; i++)
    {
        if (app_env.dev_rec[idx].svc_rec[i].attr_hdl == uuid)
        {
            r = true;
            break;
        }
    }    
#endif
    return r;
}
#endif

/**
 ****************************************************************************************
 * @brief Get the client index by connection handle and uuid.
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
uint8_t app_get_client_idx_by_conhdl(uint16_t conhdl, uint16_t uuid)
{
    uint8_t idx = 0, stop = false;

    for (; idx < BLE_CONNECTION_MAX; idx++)
    {
        switch (uuid)
        {
#if BLE_CSC_COLLECTOR
            case ATT_SVC_CYCLING_SPEED_CADENCE:
                if (app_cscpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;    
#endif
#if BLE_RSC_COLLECTOR
            case ATT_SVC_RUNNING_SPEED_CADENCE:
                if (app_rscpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;    
#endif
#if BLE_PAS_CLIENT
            case ATT_SVC_PHONE_ALERT_STATUS:
                if (app_paspc_env[idx].conhdl == conhdl)
                    stop = true;
                break;    
#endif
#if BLE_AN_CLIENT
            case ATT_SVC_ALERT_NTF:
                if (app_anpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;    
#endif
#if BLE_HT_COLLECTOR
            case ATT_SVC_HEALTH_THERMOM:
                if (app_htpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;    
#endif    
#if BLE_BP_COLLECTOR
            case ATT_SVC_BLOOD_PRESSURE:
                if (app_blpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;  
#endif 
#if BLE_HR_COLLECTOR
            case ATT_SVC_HEART_RATE:
                if (app_hrpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif 
#if BLE_GL_COLLECTOR
            case ATT_SVC_GLUCOSE:
                if (app_glpc_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif 
#if BLE_FINDME_LOCATOR
            case ATT_SVC_IMMEDIATE_ALERT:
                if (app_findl_env[idx].conhdl == conhdl)
                    stop = true;
                break; 
#endif 
#if BLE_PROX_MONITOR
            case ATT_SVC_LINK_LOSS:
                if (app_proxm_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif
#if BLE_TIP_CLIENT
            case ATT_SVC_CURRENT_TIME:
                if (app_tipc_env[idx].conhdl == conhdl)
                    stop = true;
                break; 
#endif
#if BLE_SP_CLIENT
            case ATT_SVC_SCAN_PARAMETERS:
                if (app_scppc_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif
#if BLE_DIS_CLIENT
            case ATT_SVC_DEVICE_INFO:
                if (app_disc_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif
#if BLE_BATT_CLIENT
            case ATT_SVC_BATTERY_SERVICE:
                if (app_basc_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif
#if BLE_HID_BOOT_HOST
            case ATT_SVC_HID:
                if (app_hogpbh_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif
#if BLE_HID_REPORT_HOST
            case ATT_SVC_HID:
                if (app_hogprh_env[idx].conhdl == conhdl)
                    stop = true;
                break;
#endif               
                
            default:
                QPRINTF("Unknown UUID\r\n");
                break;
        }

        if (stop == true)
            return idx;
    }

    return APP_INVALID_INDEX;
}
#endif

/**
 ****************************************************************************************
 * @brief Get the client service UUID enable status from application device record
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
uint8_t app_get_qpp_client_service_status(uint8_t idx)
{
    uint8_t enabled = 0;

#if BLE_QPP_CLIENT
    enabled = app_qppc_env[idx].enabled;
#endif

    return enabled;
}

uint8_t app_get_client_service_status(uint8_t idx, uint16_t uuid)
{
    uint8_t enabled = 0;
    switch (uuid)
    {
#if BLE_PAS_CLIENT
        case ATT_SVC_PHONE_ALERT_STATUS:
            enabled = app_paspc_env[idx].enabled;
            break;
#endif
#if BLE_AN_CLIENT
        case ATT_SVC_ALERT_NTF:
            enabled = app_anpc_env[idx].enabled;
            break;
#endif
#if BLE_HT_COLLECTOR
        case ATT_SVC_HEALTH_THERMOM:
            enabled = app_htpc_env[idx].enabled;
            break;
#endif
#if BLE_BP_COLLECTOR
        case ATT_SVC_BLOOD_PRESSURE:
            enabled = app_blpc_env[idx].enabled;
            break;
#endif
#if BLE_HR_COLLECTOR
        case ATT_SVC_HEART_RATE:
            enabled = app_hrpc_env[idx].enabled;
            break;
#endif
#if BLE_GL_COLLECTOR
        case ATT_SVC_GLUCOSE:
            enabled = app_glpc_env[idx].enabled;
            break;
#endif
#if BLE_FINDME_LOCATOR
        case ATT_SVC_IMMEDIATE_ALERT:
            enabled = app_findl_env[idx].enabled;
            break;
#endif
#if BLE_TIP_CLIENT
        case ATT_SVC_CURRENT_TIME:
        case ATT_SVC_REF_TIME_UPDATE:
        case ATT_SVC_NEXT_DST_CHANGE:
            enabled = app_tipc_env[idx].enabled;
            break;
#endif
#if BLE_PROX_MONITOR
        case ATT_SVC_LINK_LOSS:
        case ATT_SVC_TX_POWER:
            enabled = app_proxm_env[idx].enabled;
            break;
#endif
#if BLE_HID_BOOT_HOST
        case ATT_SVC_HID:
            enabled = app_hogpbh_env[idx].enabled;
            break;
#endif
#if BLE_HID_REPORT_HOST
        case ATT_SVC_HID:
            enabled = app_hogprh_env[idx].enabled;
            break;
#endif
#if BLE_SP_CLIENT
        case ATT_SVC_SCAN_PARAMETERS:
            enabled = app_scppc_env[idx].enabled;
            break;
#endif
#if BLE_DIS_CLIENT
        case ATT_SVC_DEVICE_INFO:
            enabled = app_disc_env[idx].enabled;
            break;
#endif
#if BLE_BATT_CLIENT
        case ATT_SVC_BATTERY_SERVICE:
            enabled = app_basc_env[idx].enabled;
            break;
#endif
#if BLE_CSC_COLLECTOR
        case ATT_SVC_CYCLING_SPEED_CADENCE:
            enabled = app_cscpc_env[idx].enabled;
            break;
#endif
#if BLE_RSC_COLLECTOR
        case ATT_SVC_RUNNING_SPEED_CADENCE:
            enabled = app_rscpc_env[idx].enabled;;
            break;
#endif
        default:
            break;
    }
   
    return enabled;
}
#endif

/**
 ****************************************************************************************
 * @brief Set the client service enable status from application device record
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
void app_set_qpp_client_service_status(uint16_t conhdl)
{
    uint8_t idx;

    for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
#if BLE_QPP_CLIENT
        if (conhdl == app_qppc_env[idx].conhdl)
        {
            app_qppc_env[idx].enabled = false;
        }
#endif
    }
}

void app_set_client_service_status(uint16_t conhdl)
{
    uint8_t idx;

    for (idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
#if BLE_PAS_CLIENT
        if (conhdl == app_paspc_env[idx].conhdl)
        {
            app_paspc_env[idx].enabled = false;
        }
#endif
#if BLE_AN_CLIENT
        if (conhdl == app_anpc_env[idx].conhdl)
        {
            app_anpc_env[idx].enabled = false;
        }
#endif
#if BLE_HT_COLLECTOR
        if (conhdl == app_htpc_env[idx].conhdl)
        {
            app_htpc_env[idx].enabled = false;
        }
#endif
#if BLE_BP_COLLECTOR
        if (conhdl == app_blpc_env[idx].conhdl)
        {
            app_blpc_env[idx].enabled = false;
        }
#endif
#if BLE_HR_COLLECTOR
        if (conhdl == app_hrpc_env[idx].conhdl)
        {
            app_hrpc_env[idx].enabled = false;
        }
#endif
#if BLE_GL_COLLECTOR
        if (conhdl == app_glpc_env[idx].conhdl)
        {
            app_glpc_env[idx].enabled = false;
        }
#endif
#if BLE_FINDME_LOCATOR
        if (conhdl == app_findl_env[idx].conhdl)
        {
            app_findl_env[idx].enabled = false;
        }
#endif
#if BLE_TIP_CLIENT
        if (conhdl == app_tipc_env[idx].conhdl)
        {
            app_tipc_env[idx].enabled = false;
        }
#endif
#if BLE_PROX_MONITOR
        if (conhdl == app_proxm_env[idx].conhdl)
        {
            app_proxm_env[idx].enabled = false;
        }
#endif
#if BLE_HID_BOOT_HOST
        if (conhdl == app_hogpbh_env->conhdl)
        {
            app_hogpbh_env[idx].enabled = false;
        }
#endif
#if BLE_HID_REPORT_HOST
        if (conhdl == app_hogprh_env->conhdl)
        {
            app_hogprh_env[idx].enabled = false;
        }
#endif
#if BLE_SP_CLIENT
        if (conhdl == app_scppc_env[idx].conhdl)
        {
            app_scppc_env[idx].enabled = false;
        }
#endif
#if BLE_DIS_CLIENT
        if (conhdl == app_disc_env[idx].conhdl)
        {
            app_disc_env[idx].enabled = false;
        }
#endif
#if BLE_BATT_CLIENT
        if (conhdl == app_basc_env[idx].conhdl)
        {
            app_basc_env[idx].enabled = false;
        }
#endif
#if BLE_CSC_COLLECTOR
        if (conhdl == app_cscpc_env[idx].conhdl)
        {
            app_cscpc_env[idx].enabled = false;
        }
#endif
#if BLE_RSC_COLLECTOR
        if (conhdl == app_rscpc_env[idx].conhdl)
        {
            app_rscpc_env[idx].enabled = false;
        }
#endif
    }
}
#endif

/**
 ****************************************************************************************
 * @brief Create server service database
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
void app_create_server_service_DB(void)
{
#if (BLE_AN_SERVER)
    app_anps_create_db(app_anps_env->supp_new_alert_cat, app_anps_env->supp_unread_alert_cat);
#endif
#if BLE_HT_THERMOM
    app_htpt_create_db(HTPT_MEAS_INTV_DFLT_MIN, HTPT_MEAS_INTV_DFLT_MAX, app_htpt_env->features);
#endif
#if (BLE_BP_SENSOR)
    app_blps_create_db(app_blps_env->features);
#endif
#if (BLE_DIS_SERVER)
    app_diss_create_db(app_diss_env->features);
#endif
#if BLE_FINDME_TARGET
    app_findt_create_db();
#endif
#if BLE_HR_SENSOR
    app_hrps_create_db(app_hrps_env->features);
#endif
#if BLE_PROX_REPORTER
    app_proxr_create_db(app_proxr_env->features);
#endif
#if BLE_TIP_SERVER
    app_tips_create_db(app_tips_env->features);
#endif
#if BLE_SP_SERVER
    app_scpps_create_db(app_scpps_env->features);
#endif
#if BLE_BATT_SERVER
    app_bass_create_db(app_bass_env->bas_nb, app_bass_env->features);
#endif
#if BLE_GL_SENSOR
    app_glps_create_db(0, GLPS_MEAS_CTX_SUP);
#endif
#if BLE_HID_DEVICE
    #if (BLE_BATT_SERVER==0)
    struct hogpd_hids_cfg cfg[HOGPD_NB_HIDS_INST_MAX];
    for (uint8_t i = 0; i < app_hogpd_env->hids_nb; i++)
    {
        cfg[i].features = app_hogpd_env->features[i];
        cfg[i].hid_info.bcdHID = 0x210;     // HID Class Spec release Number, for example, 2.10 is 0x210
        cfg[i].hid_info.bCountryCode = 33;  // US
        cfg[i].hid_info.flags = HIDS_REMOTE_WAKE_CAPABLE | HIDS_NORM_CONNECTABLE;
        if (app_hogpd_env->features[i].svc_features & HOGPD_CFG_MAP_EXT_REF)
        {
            #if BLE_BATT_SERVER
            cfg[i].ext_rep_ref.start_hdl = bass_env.shdl[i];
            cfg[i].ext_rep_ref.end_hdl = bass_env.shdl[i] + BAS_IDX_NB;
            #endif
            cfg[i].ext_rep_ref.uuid = ATT_SVC_BATTERY_SERVICE;
            cfg[i].ext_rep_ref_uuid = ATT_CHAR_BATTERY_LEVEL; // Battery level
        }
    }
    app_hogpd_create_db(app_hogpd_env->hids_nb, &cfg[0]);
    #endif
#endif
#if BLE_CSC_SENSOR
    app_cscps_create_db(CSCP_FEAT_WHEEL_REV_DATA_SUPP | CSCP_FEAT_CRANK_REV_DATA_SUPP | CSCP_FEAT_MULT_SENSOR_LOC_SUPP,
                        CSCPS_SENSOR_LOC_SUPP, CSCP_LOC_FRONT_WHEEL);
#endif
#if BLE_RSC_SENSOR
    app_rscps_create_db(RSCP_FEAT_ALL_SUPP, RSCPS_SENSOR_LOC_SUPP, RSCP_LOC_OTHER);
#endif
#if BLE_PAS_SERVER
    app_pasps_create_db(PASP_RINGER_ACTIVE, PASP_RINGER_SILENT);
#endif
#if BLE_OTA_SERVER
    
    #ifdef  ENAB_OTAS_SET_UUID
    app_env.app_otas_uuid_flag = app_otas_change_svc_uuid((uint8_t *)OTAS_SVC_UUID_128BIT);
    #endif
    
    #ifdef  ENAB_OTAS_SEND_DATA
    uint8_t res = app_otas_set_data_addr(FALSH_DAT_START_ADDR);
    #endif
    
    app_otas_create_db();
#endif
#if BLE_QPP_SERVER
    qpps_set_service_uuid((uint8_t *)QPP_SVC_PRIVATE_UUID);
    app_qpps_env->tx_char_num = QPPS_NOTIFY_NUM;
    #if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)
    app_qpps_env->tx_buffer_available = QPPS_TX_BUFFER_SIZE;
    #endif
    app_qpps_create_db(app_qpps_env->tx_char_num);
#endif
}
#endif

/**
 ****************************************************************************************
 * @brief Enable server service
 *
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
void app_enable_server_service(uint8_t enabled, uint16_t conhdl)
{
#if BLE_AN_SERVER
    if (enabled == true)
    {
        app_anps_env->conhdl = conhdl;
        app_anps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, PRF_CLI_STOP_NTFIND,
                            PRF_CLI_STOP_NTFIND);
        app_anps_env->enabled = true;
    }
    else if (app_anps_env->enabled == true)
    {
        app_anps_env->enabled = false;
    }
#endif
#if BLE_HT_THERMOM
    if (enabled == true)
    {
        app_htpt_env->conhdl = conhdl;
        app_htpt_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, PRF_CLI_STOP_NTFIND,
                            PRF_CLI_STOP_NTFIND, PRF_CLI_STOP_NTFIND, APP_HTPT_MEAS_INTV);
        app_htpt_env->enabled = true;
    }
    else if (app_htpt_env->enabled == true)
    {
        app_htpt_env->enabled = false;
    }
#endif
#if BLE_BP_SENSOR
    if (enabled == true)
    {
        app_blps_env->conhdl = conhdl;
        app_blps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, PRF_CLI_STOP_NTFIND, PRF_CLI_STOP_NTFIND, 
                            BPS_F_BODY_MVMT_DETECT_SUPPORTED | BPS_F_CUFF_FIT_DETECT_SUPPORTED);
        app_blps_env->enabled = true; 
    }
    else if (app_blps_env->enabled == true)
    {
        app_blps_env->enabled = false;
    }
#endif
#if BLE_HR_SENSOR
    if (enabled == true)
    {
        app_hrps_env->conhdl = conhdl;
        app_hrps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, PRF_CLI_STOP_NTFIND, 
                            /* app_hrps_env.features & HRPS_BODY_SENSOR_LOC_CHAR_SUP */ HRS_LOC_CHEST);
        app_hrps_env->enabled = true;
    }
    else if (app_hrps_env->enabled == true)
    {
        app_hrps_env->enabled = false;
    }
#endif
#if BLE_GL_SENSOR
    if (enabled == true)
    {
        app_glps_env->conhdl = conhdl;
        app_glps_enable_req(conhdl, GLP_FET_LOW_BAT_DET_DUR_MEAS_SUPP |
                            GLP_FET_SENS_MFNC_DET_SUPP |
                            GLP_FET_SENS_SPL_SIZE_SUPP |
                            GLP_FET_SENS_STRIP_INSERT_ERR_DET_SUPP |
                            GLP_FET_SENS_STRIP_TYPE_ERR_DET_SUPP |
                            GLP_FET_SENS_RES_HIGH_LOW_DET_SUPP,
                            PERM_RIGHT_ENABLE, PRF_CON_NORMAL, app_glps_env->evt_cfg);
    }
    else if (app_glps_env->enabled == true)
    {
        app_glps_env->enabled = false;
    }
#endif
#if BLE_PROX_REPORTER
    if (enabled == true)
    {
        const int8_t pa_gain2dbm_table[] = {-20, -18, -16, -14, -12, -10, -8, -6, -4, -2, 0, 2, 4};
#if !QN_WORK_MODE
        const int8_t pa_gain = rf_tx_power_level_get();
#else
        const int8_t pa_gain = 10;
#endif
        app_proxr_env->conhdl = conhdl;
        app_proxr_enable_req(conhdl, PERM_RIGHT_ENABLE, PROXR_ALERT_NONE, pa_gain2dbm_table[pa_gain]);
        app_proxr_env->enabled = true;
    }
    else if (app_proxr_env->enabled == true)
    {
        app_proxr_env->enabled = false;
    }
#endif
#if BLE_FINDME_TARGET
    if (enabled == true)
    {
        app_findt_env->conhdl = conhdl;
        app_findt_enable_req(conhdl, PERM_RIGHT_ENABLE);
        app_findt_env->enabled = true;
    }
    else if (app_findt_env->enabled == true)
    {
        app_findt_env->enabled = false;
    }
#endif
#if BLE_TIP_SERVER
    if (enabled == true)
    {
        app_tips_env->conhdl = conhdl;
        app_tips_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, PRF_CLI_STOP_NTFIND);
        app_tips_env->enabled = true;        
    }
    else if (app_tips_env->enabled == true)
    {
        app_tips_env->enabled = false;
    }
#endif
#if BLE_HID_DEVICE
    if (enabled == true)
    {
        struct hogpd_hids_ntf_cfg ntf_cfg;
        ntf_cfg.boot_kb_in_report_ntf_en = PRF_CLI_STOP_NTFIND;
        ntf_cfg.boot_mouse_in_report_ntf_en = PRF_CLI_STOP_NTFIND;
        for (uint8_t i = 0; i < HOGPD_NB_REPORT_INST_MAX; i++)
            ntf_cfg.report_ntf_en[i] = PRF_CLI_STOP_NTFIND;

        app_hogpd_env->conhdl = conhdl;
        app_hogpd_env->enabled = true;
        app_hogpd_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, HOGPD_NB_HIDS_INST_MAX,
                             &ntf_cfg);
    }
    else if (app_hogpd_env->enabled == true)
    {
        app_hogpd_env->enabled = false;
    }
#endif
#if BLE_DIS_SERVER
    if (enabled == true)
    {
        app_diss_env->conhdl = conhdl;
        app_diss_enable_req(conhdl, PERM_RIGHT_ENABLE);
        app_diss_env->enabled = true;
    }
    else if (app_diss_env->enabled == true)
    {
        app_diss_env->enabled = false;
    }
#endif
#if BLE_SP_SERVER
    if (enabled == true)
    {
        app_scpps_env->conhdl = conhdl;
        app_scpps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, PRF_CLI_STOP_NTFIND);
        app_scpps_env->enabled = true;
    }
    else if (app_scpps_env->enabled == true)
    {
        app_scpps_env->enabled = false;
    }
#endif
#if BLE_BATT_SERVER
    if (enabled == true)
    {
        app_bass_env->conhdl = conhdl;
        app_bass_enable_req(conhdl, app_bass_env->bas_nb, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, 
                            app_bass_env->batt_level_ntf_cfg, app_bass_env->old_batt_lvl, 
                            app_bass_env->current_batt_lvl, app_bass_env->batt_level_pres_format);
        app_bass_env->enabled = true;
    }
    else if (app_bass_env->enabled == true)
    {
        app_bass_env->enabled = false;
    }
#endif
#if BLE_CSC_SENSOR
    if (enabled == true)
    {
        app_cscps_env->conhdl = conhdl;
        app_cscps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, 
                            app_cscps_env->app_cfg & CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND, 
                            app_cscps_env->app_cfg & CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND,
                            0);
        app_cscps_env->enabled = true;
    }
    else if (app_cscps_env->enabled == true)
    {
        app_cscps_env->enabled = false;
    }
#endif
#if BLE_RSC_SENSOR
    if (enabled == true)
    {
        app_rscps_env->conhdl = conhdl;
        app_rscps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, 
                             app_rscps_env->app_cfg & RSCP_PRF_CFG_FLAG_RSC_MEAS_NTF ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND, 
                             app_rscps_env->app_cfg & RSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND);
        app_rscps_env->enabled = true;
    }
    else if (app_rscps_env->enabled == true)
    {
        app_rscps_env->enabled = false;
    }
#endif
#if BLE_PAS_SERVER
    if (enabled == true)
    {
        app_pasps_env->conhdl = conhdl;
        app_pasps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL, 
                             app_pasps_env->app_cfg & PASPS_FLAG_ALERT_STATUS_CFG ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND, 
                             app_pasps_env->app_cfg & PASPS_FLAG_RINGER_SETTING_CFG ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND);
        app_pasps_env->enabled = true;
    }
    else if (app_pasps_env->enabled == true)
    {
        app_pasps_env->enabled = false;
    }
#endif

#if BLE_OTA_SERVER
    if (enabled == true)
        app_otas_enable_req(conhdl, PERM_RIGHT_ENABLE);
    else
        app_otas_disable_req(conhdl);
#endif
#if BLE_QPP_SERVER
    if (enabled == true)
    {
        app_qpps_env->conhdl = conhdl;
        app_qpps_enable_req(conhdl, PERM_RIGHT_ENABLE, PRF_CON_NORMAL,  PRF_CLI_STOP_NTFIND);
        app_qpps_env->enabled = true;
    }
    else if (app_qpps_env->enabled == true)
    {
        app_qpps_env->enabled = false;
    }
#endif
}
#endif

/**
 ****************************************************************************************
 * @brief Set Advertising data
 *
 * @note
 * The format of Advertising data and Scan Response data contains a sequence of AD structures.
 * Each AD structure shall have a Length field of one octet, which contains the Length value, 
 * and a Data field of Length octets. The first octet of the Data field contains the AD type field.
 * The content of the remaining Length C 1 octet in the Data field depends on the value of 
 * the AD type field and is called the AD data.
 * More details please see Core spec 4.0 page 1735.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
uint8_t app_set_adv_data(uint16_t disc_mode)
{
    uint8_t len;
    /* "\x02\x01\x0?\x08\x08NXP BLE" */
    
    // Advertising data, BLE only, general discovery mode and short device name
    app_env.adv_data[0] = 0x02;
    app_env.adv_data[1] = GAP_AD_TYPE_FLAGS;
    if (GAP_GEN_DISCOVERABLE == (disc_mode & GAP_GEN_DISCOVERABLE))
    {
        /// Mode in general discoverable
        app_env.adv_data[2] = 0x06;
    }
    else if (GAP_LIM_DISCOVERABLE == (disc_mode & GAP_LIM_DISCOVERABLE))
    {
        /// Mode in limited discoverable
        app_env.adv_data[2] = 0x05;
    }
    else
    {
        /// Mode in non-discoverable
        app_env.adv_data[2] = 0x04;
    }
#if (defined(CFG_EACI))
    app_env.adv_data[3] = device_name.namelen + 1;
    app_env.adv_data[4] = 0x09;
    memcpy((char *)&app_env.adv_data[5], device_name.name, device_name.namelen);
    len = 5 + device_name.namelen;
#else
    nvds_tag_len_t name_length = 31 - 5; // The maximum length of Advertising data is 31 Octets

    if (NVDS_OK != nvds_get(NVDS_TAG_DEVICE_NAME, &name_length, &app_env.adv_data[5]))
    {
        // NVDS is empty, use default name
        name_length = strlen(QN_LOCAL_NAME);
        strcpy((char *)&app_env.adv_data[5], QN_LOCAL_NAME);
    }
    else
    {
        name_length--; // Discard '\0'
    }
    app_env.adv_data[3] = name_length + 1;
    app_env.adv_data[4] = GAP_AD_TYPE_SHORTENED_NAME;
    len = 5 + name_length;
#endif
    
    return len;
}
#endif

/**
 ****************************************************************************************
 * @brief Set Scan response data
 *
 * @note
 * The format of Scan Response data is the same as the same as Advertising data.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
uint8_t app_set_scan_rsp_data(uint16_t srv_flag)
{
    /* "\x05\x12\x06\x00\x80\x0C" */
    uint8_t len;

    if (srv_flag == 0)
    {
        // Slave connection interval range:0x12
        app_env.scanrsp_data[0] = 0x05;
        app_env.scanrsp_data[1] = 0x12;
        app_env.scanrsp_data[2] = 0x06;
        app_env.scanrsp_data[3] = 0x00;
        app_env.scanrsp_data[4] = 0x80;
        app_env.scanrsp_data[5] = 0x0C;

        len = 6;
    }
    else {
#if ( BLE_AN_SERVER || BLE_CSC_SENSOR || BLE_PAS_SERVER || BLE_HT_THERMOM || BLE_RSC_SENSOR\
      || BLE_BP_SENSOR || BLE_HR_SENSOR || BLE_GL_SENSOR || BLE_TIP_SERVER || BLE_SP_SERVER\
      || BLE_HID_DEVICE || BLE_PROX_REPORTER || BLE_FINDME_TARGET || BLE_DIS_SERVER || BLE_BATT_SERVER\
      || BLE_OTA_SERVER )
    uint8_t remain_len = 25-2;
#endif
    bool complete = true;
    len = 2;

#if BLE_QPP_SERVER
    if (srv_flag & BLE_QPPS_SERVER_BIT)
    {
        app_env.scanrsp_data[0] = ATT_UUID_128_LEN + 1;
        app_env.scanrsp_data[1] = GAP_AD_TYPE_MORE_128_BIT_UUID;
        memcpy(app_env.scanrsp_data + 2, QPP_SVC_PRIVATE_UUID, ATT_UUID_128_LEN);
        return (ATT_UUID_128_LEN + 2);
    }
#endif
#if BLE_AN_SERVER
    if (len <= remain_len)
    {
         if (srv_flag & BLE_AN_SERVER_BIT)
         {
             app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_ALERT_NTF & 0x00FF);
             app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_ALERT_NTF >> 8);
             len += 2;
         }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_HT_THERMOM
    if (len <= remain_len)
    {
         if (srv_flag & BLE_HT_THERMOM_BIT)
         {
             app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_HEALTH_THERMOM&0x00FF);
             app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_HEALTH_THERMOM>>8);
             len += 2;
         }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_BP_SENSOR
    if (len <= remain_len)
    {
        if (srv_flag & BLE_BP_SENSOR_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_BLOOD_PRESSURE&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_BLOOD_PRESSURE>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_HR_SENSOR
    if (len <= remain_len)
    {
        if (srv_flag & BLE_HR_SENSOR_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_HEART_RATE&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_HEART_RATE>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_GL_SENSOR
    if (len <= remain_len)
    {
        if (srv_flag & BLE_GL_SENSOR_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_GLUCOSE&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_GLUCOSE>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_TIP_SERVER
    if (len <= remain_len-4)
    {
        if (srv_flag & BLE_TIP_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_CURRENT_TIME&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_CURRENT_TIME>>8);
            app_env.scanrsp_data[len+2] = (uint8_t)(ATT_SVC_REF_TIME_UPDATE&0x00FF);
            app_env.scanrsp_data[len+3] = (uint8_t)(ATT_SVC_REF_TIME_UPDATE>>8);
            app_env.scanrsp_data[len+4] = (uint8_t)(ATT_SVC_NEXT_DST_CHANGE&0x00FF);
            app_env.scanrsp_data[len+5] = (uint8_t)(ATT_SVC_NEXT_DST_CHANGE>>8);
            len += 6;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_SP_SERVER
    if (len <= remain_len)
    {
        if (srv_flag & BLE_SP_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_SCAN_PARAMETERS&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_SCAN_PARAMETERS>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_HID_DEVICE
    if (len <= remain_len)
    {
        if (srv_flag & BLE_HID_DEVICE_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_HID&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_HID>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_PROX_REPORTER
    if (len <= remain_len-4)
    {
        if (srv_flag & BLE_PROX_REPORTER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_LINK_LOSS&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_LINK_LOSS>>8);
            app_env.scanrsp_data[len+2] = (uint8_t)(ATT_SVC_IMMEDIATE_ALERT&0x00FF);
            app_env.scanrsp_data[len+3] = (uint8_t)(ATT_SVC_IMMEDIATE_ALERT>>8);
            app_env.scanrsp_data[len+4] = (uint8_t)(ATT_SVC_TX_POWER&0x00FF);
            app_env.scanrsp_data[len+5] = (uint8_t)(ATT_SVC_TX_POWER>>8);
            len += 6;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if (BLE_FINDME_TARGET && !BLE_PROX_REPORTER)
    if (len <= remain_len)
    {
        if ((srv_flag & BLE_FINDME_TARGET_BIT) && !(srv_flag & BLE_PROX_REPORTER_BIT))
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_IMMEDIATE_ALERT&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_IMMEDIATE_ALERT>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_DIS_SERVER
    if (len <= remain_len)
    {
        if (srv_flag & BLE_DIS_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_DEVICE_INFO&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_DEVICE_INFO>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_BATT_SERVER
    if (len <= remain_len)
    {
        if (srv_flag & BLE_BATT_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_BATTERY_SERVICE&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_BATTERY_SERVICE>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_CSC_SENSOR
    if (len <= remain_len)
    {
        if (srv_flag & BLE_CSCP_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_CYCLING_SPEED_CADENCE&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_CYCLING_SPEED_CADENCE>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_PAS_SERVER
    if (len <= remain_len)
    {
        if (srv_flag & BLE_PASP_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_PHONE_ALERT_STATUS&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_PHONE_ALERT_STATUS>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
#if BLE_RSC_SENSOR
    if (len <= remain_len)
    {
        if (srv_flag & BLE_RSCP_SERVER_BIT)
        {
            app_env.scanrsp_data[len+0] = (uint8_t)(ATT_SVC_RUNNING_SPEED_CADENCE&0x00FF);
            app_env.scanrsp_data[len+1] = (uint8_t)(ATT_SVC_RUNNING_SPEED_CADENCE>>8);
            len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
    
#if BLE_OTA_SERVER
    if (len <= remain_len)
    {
        #ifdef ENAB_OTAS_SET_UUID
        if(app_env.app_otas_uuid_flag != 0)
        {
            app_env.scanrsp_data[0] = ATT_UUID_128_LEN + 1;
            app_env.scanrsp_data[1] = GAP_AD_TYPE_MORE_128_BIT_UUID;
            memcpy(app_env.scanrsp_data + 2, OTAS_SVC_UUID_128BIT, ATT_UUID_128_LEN);
            return (ATT_UUID_128_LEN + 2);
        }
        else
        #endif
        {
        app_env.scanrsp_data[len+0] = (uint8_t)(OTAS_SVC_PRIVATE_UUID & 0x00FF);
        app_env.scanrsp_data[len+1] = (uint8_t)(OTAS_SVC_PRIVATE_UUID >> 8);
        len += 2;
        }
    }
    else
    {
        complete = false;
    }
#endif
 
// in case no profile server enable
    if (len == 2)
    {
        app_env.scanrsp_data[2] = (uint8_t)(ATT_SVC_GENERIC_ACCESS&0x00FF);
        app_env.scanrsp_data[3] = (uint8_t)(ATT_SVC_GENERIC_ACCESS>>8);
        app_env.scanrsp_data[4] = (uint8_t)(ATT_SVC_GENERIC_ATTRIBUTE&0x00FF);
        app_env.scanrsp_data[5] = (uint8_t)(ATT_SVC_GENERIC_ATTRIBUTE>>8);
        len += 4;
    }
    // UUID list length
    app_env.scanrsp_data[0] = len - 1;

    // Complete UUID list or not
    if (complete == true)
        app_env.scanrsp_data[1] = 0x03;
    else
        app_env.scanrsp_data[1] = 0x02;
    
    // Slave connection interval range:0x12
    app_env.scanrsp_data[len+0] = 0x05;
    app_env.scanrsp_data[len+1] = 0x12;
    app_env.scanrsp_data[len+2] = 0x06;
    app_env.scanrsp_data[len+3] = 0x00;
    app_env.scanrsp_data[len+4] = 0x80;
    app_env.scanrsp_data[len+5] = 0x0C;

    len += 6; }
    
    return len;
}
#endif

/**
 ****************************************************************************************
 * @brief Parser Advertising or Scan response data
 *
 ****************************************************************************************
 */
#if (BLE_CENTRAL || BLE_OBSERVER)
bool app_parser_adv_data(uint8_t *pdata, uint8_t total_len, struct app_adv_data *padv)
{
    uint8_t *pend = pdata + total_len;

    memset(padv, 0, sizeof(struct app_adv_data));
    while (pdata < pend)
    {
        uint8_t data_len = *pdata++; // Data length include advertising type

        if ((pdata + data_len) > pend)
            return false;

        switch (*pdata)
        {
            case GAP_AD_TYPE_SHORTENED_NAME:
            case GAP_AD_TYPE_COMPLETE_NAME:
                memcpy(padv->name, pdata + 1, data_len - 1);
                padv->name[data_len] = '\0';
                padv->flag |= AD_TYPE_NAME_BIT;
                break;
            case GAP_AD_TYPE_MORE_16_BIT_UUID:
            case GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
            {
                uint16_t *puuid = &(padv->uuid[0]);
                for (uint8_t i = 0; i < (data_len - 1); i += 2)
                {
                   *puuid++ = (uint16_t)(pdata[i + 1] | (pdata[i + 2] << 8));
                    padv->uuid_num++;
                }
                padv->flag |= AD_TYPE_16bitUUID_BIT;
                break;
            }
            default:
                break;
        }
        pdata += data_len;
    }

    return true;
}
#endif

/**
 ****************************************************************************************
 * @brief Check Updated Connection Parameters is acceptable or not
 *
 ****************************************************************************************
 */
bool app_check_update_conn_param(struct gap_conn_param_update const *conn_param)
{
    return true;
}

/**
 ****************************************************************************************
 * @brief Configure ble new features
 ****************************************************************************************
 */
#if ((QN_MULTI_NOTIFICATION_IN_ONE_EVENT) || (QN_READ_INDICATION) || QN_SLAVE_LATENCY_IMPROVEMENT)
void ble_new_feature(void)
{
#if ((QN_MULTI_NOTIFICATION_IN_ONE_EVENT) || (QN_READ_INDICATION))
    static struct ke_msg_handler atts_default_state_new[16];
    static struct ke_state_handler atts_default_handler_new = KE_STATE_HANDLER(atts_default_state_new);
#endif

#if ((QN_MULTI_NOTIFICATION_IN_ONE_EVENT) && (defined(CFG_9020_B4)))
    static struct ke_msg_handler gatt_idle_new[24];
    static struct ke_state_handler gatt_idle_handler = KE_STATE_HANDLER(gatt_idle_new);
#endif

    struct qn_feature_ctrl ctrl;

    ctrl.flag = 0;
#if ((QN_MULTI_NOTIFICATION_IN_ONE_EVENT) || (QN_READ_INDICATION))
    ctrl.atts_default_state_new = atts_default_state_new;
    ctrl.atts_default_handler_new = &atts_default_handler_new;
#endif

#if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)
    ctrl.flag |= 0x01;
    ctrl.atts_hdl_val_ntf_req_handler_patch = (ke_msg_func_t)atts_hdl_val_ntf_req_handler_patch;
    #if (defined(CFG_9020_B4))
    ctrl.gatt_idle_new = gatt_idle_new;
    ctrl.gatt_idle_handler_new = &gatt_idle_handler;
    #endif
    #if (defined(CFG_9020_B2))
    ctrl.l2cc_data_send_rsp_att_handler_patch = (ke_msg_func_t)l2cc_data_send_rsp_att_handler_patch;
    #endif
#endif
    
#if (QN_READ_INDICATION)
    ctrl.flag |= 0x02;
    ctrl.l2cc_data_packet_ind_handler_patch = (ke_msg_func_t)atts_l2cc_data_packet_ind_handler_patch;
    ctrl.gatt_read_req_cfm_handler_patch = (ke_msg_func_t)gatt_read_req_cfm_handler_patch;
#endif

#if (QN_SLAVE_LATENCY_IMPROVEMENT)
    ctrl.flag |= 0x4;
    ctrl.llcp_con_up_req_handler_patch = (ke_msg_func_t)llcp_con_up_req_handler_patch;
    ctrl.llcp_channel_map_req_handler_patch = llcp_channel_map_req_handler_patch;
    #if defined(QN_9020_B2)
    ctrl.lld_evt_schedule_patch = lld_evt_schedule_patch;
    ctrl.lld_evt_end_patch = lld_evt_end_patch;
    #endif
    #if defined(QN_9020_B4)
    ctrl.lld_evt_restart_patch = lld_evt_restart_patch;
    #endif
    ctrl.lld_evt_schedule_next = lld_evt_schedule_next;
#endif

    qn_feature_config(&ctrl);
}
#endif



