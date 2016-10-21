/**
 ****************************************************************************************
 *
 * @file app_gap_task.c
 *
 * @brief Application GAP task implementation
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
 * @addtogroup APP_GAP_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#include "lib.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief calibrate rssi
 ****************************************************************************************
 */
int8_t app_correct_rssi(int8_t rssi)
{
    int8_t accurate_rssi;
    
    if(rssi <= -17)
    {
        // 1.026 * rssi + 8.641 + 0.5;
        accurate_rssi = rssi + 9;
    }
    else if(rssi <= -4)
    {
        accurate_rssi = rssi + 11;
    }
    else
    {
        accurate_rssi = rssi + 6;
    }

    return accurate_rssi;
}

/*
 ****************************************************************************************
 * @brief application setuo
 ****************************************************************************************
 */
void app_device_setup(ke_msg_id_t const msgid, void const *param)
{
    app_menu_cb(msgid, (void *)param);
    /*  Initialize the BLE device with the following device setup sequence */
    /// Set security mode
    /// Read Link and Host layer versions
    /// Read BD address
    /// Set Reconnection Address feature, only for central
    /// Set the peer Privacy Flag feature, only for central    

    // GAP_SEC1_AUTH_PAIR_ENC, need MITM, input and output ability
    app_gap_set_sec_req(QN_SEC_MODE_LEVEL);

#if (QN_32K_RCO)
    ke_timer_set(APP_SYS_RCO_CAL_TIMER, TASK_APP, 100);
#endif    
}

/*
 ****************************************************************************************
 * @brief Handles ready indication from the GAP.    *//**
 *
 * @param[in] msgid     GAP_READY_EVT
 * @param[in] param     NULL pointer
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the BLE stack initialization has completed.
 ****************************************************************************************
 */
int app_gap_ready_evt_handler(ke_msg_id_t const msgid, void const *param,
                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
#if (QN_WORK_MODE == WORK_MODE_SOC)
    app_device_setup(msgid, param);
#endif

    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles device reset complete event from the GAP.        *//**
 *
 * @param[in] msgid     GAP_RESET_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_reset_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that device reset has completed.
 ****************************************************************************************
 */
int app_gap_reset_req_cmp_handler(ke_msg_id_t const msgid, struct gap_reset_req_cmp_evt const *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
#if (QN_WORK_MODE != WORK_MODE_SOC)
    app_device_setup(msgid, param);
#endif

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles set device name complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_SET_DEVNAME_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the set device name request has completed.
 ****************************************************************************************
 */
int app_gap_set_devname_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Set device name complete\r\n");
    
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles set security request complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_SET_SEC_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_set_sec_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the set security mode has completed.
 ****************************************************************************************
 */
int app_gap_set_sec_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_set_sec_req_cmp_evt const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (APP_INIT == ke_state_get(TASK_APP))
    {
        app_gap_read_ver_req();
    }
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles read version request complete event from the GAP.        *//**
 *
 * @param[in] msgid     GAP_READ_VER_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_read_ver_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the read the version information of the stack has completed.
 ****************************************************************************************
 */
int app_gap_read_ver_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_read_ver_req_cmp_evt const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (APP_INIT == ke_state_get(TASK_APP))
    {
        app_menu_cb(msgid, (void *)param);
        app_gap_read_bdaddr_req();
    }
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles read bdaddr request complete event from the GAP.        *//**
 *
 * @param[in] msgid     GAP_READ_BDADDR_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_read_bdaddr_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the read the Bluetooth Address of the device has completed.
 ****************************************************************************************
 */
int app_gap_read_bdaddr_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_read_bdaddr_req_cmp_evt const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (APP_INIT == ke_state_get(TASK_APP))
    {
        if (param->status == CO_ERROR_NO_ERROR)
        {
            app_menu_cb(msgid, (void *)param);
        }
        else
            QPRINTF("QN BLE device setup failed.\r\n");

        // Auto Create DB here, for Server init phase 1
#if (BLE_PERIPHERAL)
        app_create_server_service_DB();
#endif
#if (QN_SECURITY_ON)
        app_init_local_smp_key();
#else
        app_gap_set_bondable_mode_req();
#endif   
    }
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles inquiry result from the GAP.        *//**
 *
 * @param[in] msgid     GAP_DEV_INQ_RESULT_EVT
 * @param[in] param     Pointer to struct gap_dev_inq_result_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to indicate that a BLE device has responded so far during inquiry process.
 * The RSSI in the adv report should be corrected by APP if this value is used.
 * Real-RSSI = RSSI + 7
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_dev_inq_result_handler(ke_msg_id_t const msgid,
                                   struct gap_dev_inq_result_evt const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    bool found = false;
    const int8_t rssi = app_correct_rssi(param->adv_rep.rssi);
    
    for (uint8_t i = 0; i < app_env.inq_idx; i++)
    {
        if (true == co_bt_bdaddr_compare(&app_env.inq_addr[i], &param->adv_rep.adv_addr))
        {
            found = true;
            break;
        }
    }

    /* add the device in the address keeper */
    if (!found && (app_env.inq_idx < BLE_CONNECTION_MAX))
    {
        struct app_adv_data adv_data;
        
        app_env.addr_type[app_env.inq_idx] = param->adv_rep.adv_addr_type;
        memcpy(app_env.inq_addr[app_env.inq_idx].addr, param->adv_rep.adv_addr.addr, BD_ADDR_LEN);
        QPRINTF("%d. %c %02X%02X%02X%02X%02X%02X", 
            app_env.inq_idx,
            app_env.addr_type[app_env.inq_idx] ? 'R' : 'P', 
            app_env.inq_addr[app_env.inq_idx].addr[5],
            app_env.inq_addr[app_env.inq_idx].addr[4],
            app_env.inq_addr[app_env.inq_idx].addr[3],
            app_env.inq_addr[app_env.inq_idx].addr[2],
            app_env.inq_addr[app_env.inq_idx].addr[1],
            app_env.inq_addr[app_env.inq_idx].addr[0]);

        app_parser_adv_data((uint8_t *)param->adv_rep.data, param->adv_rep.data_len, &adv_data);
        if (adv_data.flag & AD_TYPE_NAME_BIT)
        {
            QPRINTF(" %s", adv_data.name);
        }
        QPRINTF("\r\n");
        app_env.inq_idx++;

        app_task_msg_hdl(msgid, param);
    }

    return(KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles inquiry result from the GAP.        *//**
 *
 * @param[in] msgid     GAP_ADV_REPORT_EVT
 * @param[in] param     Pointer to struct gap_adv_report_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to indicate that a BLE device has responded so far during inquiry process.
 ****************************************************************************************
 */
#if (BLE_OBSERVER)
int app_gap_dev_scan_result_handler(ke_msg_id_t const msgid,
                                   struct gap_adv_report_evt const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    struct app_adv_data adv_data;
    
    QPRINTF("%d. %c %02X%02X%02X%02X%02X%02X", 
        app_env.inq_idx,
        param->evt.adv_rep[0].adv_addr_type ? 'R' : 'P', 
        param->evt.adv_rep[0].adv_addr.addr[5],
        param->evt.adv_rep[0].adv_addr.addr[4],
        param->evt.adv_rep[0].adv_addr.addr[3],
        param->evt.adv_rep[0].adv_addr.addr[2],
        param->evt.adv_rep[0].adv_addr.addr[1],
        param->evt.adv_rep[0].adv_addr.addr[0]);

    app_parser_adv_data((uint8_t *)param->evt.adv_rep[0].data, param->evt.adv_rep[0].data_len, &adv_data);
    if (adv_data.flag & AD_TYPE_NAME_BIT)
    {
        QPRINTF(" %s", adv_data.name);
    }
    QPRINTF("\r\n");
    app_env.inq_idx++;

    return(KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles inquiry complete from the GAP.        *//**
 *
 * @param[in] msgid     GAP_DEV_INQ_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the inquiry process has completed.
 ****************************************************************************************
 */
#if (BLE_CENTRAL || BLE_OBSERVER)
int app_gap_dev_inq_cmp_handler(ke_msg_id_t const msgid,
                                struct gap_event_common_cmd_complete const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("Total %d devices found.\r\n", app_env.inq_idx);
    ke_state_set(TASK_APP, APP_IDLE);
    app_task_msg_hdl(msgid, param);

    return(KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles inquiry cancel complete from the GAP.        *//**
 *
 * @param[in] msgid     GAP_SCAN_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the scan command has completed.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_scan_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Inquiry cancel result is 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles set mode complete event from the GAP.        *//**
 *
 * @param[in] msgid     GAP_SET_MODE_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that set mode command has completed.
 ****************************************************************************************
 */
int app_gap_set_mode_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);

    switch (ke_state_get(dest_id))
    {
    case APP_INIT:
#if (QN_PRIV_EN && BLE_PERIPHERAL)
        app_gap_set_privacy_req(PH_PRIV_EN, 0x1, 0x1);
#endif

#if QN_DEMO_AUTO
        // Created DB should has been finished by each profile service,
        // Start Adv mode automatically here
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE, 
        app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE), 
        app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
        GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
#endif
        // Set App to IDLE status here, bondale complete
        ke_state_set(TASK_APP, APP_IDLE);
        QPRINTF("QN BLE is ready.\r\n");
        break;
    case APP_IDLE:
        // Advertising started
        QPRINTF("Advertising start.\r\n");
        ke_state_set(TASK_APP, APP_ADV);

        break;
    case APP_ADV:
        // Updated adversting parameters be used
        QPRINTF("Advertsing parameter updated.\r\n");
        break;
    default:
        break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles advertising request complete event from the GAP.        *//**
 *
 * @param[in] msgid     GAP_ADV_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that stop advertising has completed.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
int app_gap_adv_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Adv stop with result: 0x%x.\r\n", param->status);
    ke_state_set(TASK_APP, APP_IDLE);

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles create connection request complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_LE_CREATE_CONN_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_le_create_conn_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the outcome of connection establishment.
 ****************************************************************************************
 */
#if (!BLE_BROADCASTER && !BLE_OBSERVER)
int app_gap_le_create_conn_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_le_create_conn_req_cmp_evt const *param,
                                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Connection with %02X%02X%02X%02X%02X%02X result is 0x%x.\r\n", 
        param->conn_info.peer_addr.addr[5],
        param->conn_info.peer_addr.addr[4],
        param->conn_info.peer_addr.addr[3],
        param->conn_info.peer_addr.addr[2],
        param->conn_info.peer_addr.addr[1],
        param->conn_info.peer_addr.addr[0],
        param->conn_info.status);

    if (APP_ADV == ke_state_get(TASK_APP))
    {
        ke_state_set(TASK_APP, APP_IDLE);
        app_set_role(GAP_PERIPHERAL_SLV);
    }
    else
    {
        app_set_role(GAP_CENTRAL_MST);
    }
    restore_ble_dev_mode_flag();
 
    if (param->conn_info.status == CO_ERROR_NO_ERROR)
    {
        app_set_link_status_by_conhdl(param->conn_info.conhdl, &param->conn_info, true);

        // Enable service here, for Server init phase 2
#if (BLE_PERIPHERAL)
        app_enable_server_service(true, param->conn_info.conhdl);
#endif
    }

    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles cancel create connection complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_CANCEL_CONN_REQ_CMP_EVT
 * @param[in] param     NULL
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the outcome of cancel connection.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_cancel_conn_req_cmp_evt_handler(ke_msg_id_t const msgid, void const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_DISCON_CMP_EVT
 * @param[in] param     Pointer to struct gap_discon_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the BLE link has been disconnected.
 ****************************************************************************************
 */
#if (!BLE_BROADCASTER && !BLE_OBSERVER)
int app_gap_discon_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_discon_cmp_evt const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        struct bd_addr peer_addr;
        app_get_bd_addr_by_conhdl(param->conhdl, &peer_addr);
        QPRINTF("Disconnect with %02X%02X%02X%02X%02X%02X reason is 0x%x.\r\n", 
                    peer_addr.addr[5],
                    peer_addr.addr[4],
                    peer_addr.addr[3],
                    peer_addr.addr[2],
                    peer_addr.addr[1],
                    peer_addr.addr[0],
                    param->reason);
    
        app_set_link_status_by_conhdl(param->conhdl, NULL, false);
        #if (BLE_CENTRAL)
        app_set_client_service_status(param->conhdl);
        #endif
        #if (BLE_PERIPHERAL)
        app_enable_server_service(false, param->conhdl);
        #endif
    
        #if QN_DEMO_AUTO
        // Start Advertising Mode
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE, 
        app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE), 
        app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
        GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
        #endif
    }
    else
    {
        QPRINTF("Disconnect status error(%#x).\r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles name request complete from the GAP.    *//**
 *
 * @param[in] msgid     GAP_NAME_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_name_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the name request has completed.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_name_req_cmp_handler(ke_msg_id_t const msgid, struct gap_name_req_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("NAME: ");
        if (param->bdname.namelen == 0)
        {
            QPRINTF("NULL");
        }
        else
        {
            for (uint8_t i = 0; i < param->bdname.namelen; i++)
                QPRINTF("%c", param->bdname.name[i]);
        }
        QPRINTF("\r\n");
    }
    else
         QPRINTF("Get Name complete with status: 0x%02X.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles read the white list size of the local device complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_LE_RD_WLST_SIZE_CMD_CMP_EVT
 * @param[in] param     Pointer to struct gap_rd_wlst_size_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the read white list size command has completed.
 ****************************************************************************************
 */
#if (QN_WL_SUPPORT)
int app_gap_le_rd_wlst_size_cmd_cmp_handler(ke_msg_id_t const msgid, struct gap_rd_wlst_size_cmd_complete const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Read white list size complete: %d.\r\n", param->wlst_size);
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles add device to white list complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_LE_ADD_DEV_TO_WLST_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the outcome of add device to white list.
 ****************************************************************************************
 */
#if (QN_WL_SUPPORT)
int app_gap_le_add_dev_to_wlst_req_cmp_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Add device white list with status: %d.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles remove device from white list complete from the GAP.    *//**
 *
 * @param[in] msgid     GAP_LE_RMV_DEV_FRM_WLST_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_event_common_cmd_complete
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the outcome of remove device from white list.
 ****************************************************************************************
 */
#if (QN_WL_SUPPORT)
int app_gap_le_rmv_dev_from_wlst_req_cmp_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Remove device white list with status: %d.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles read remote features complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_LE_RD_REMOTE_FEAT_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_le_rd_remote_feat_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that read remote features command has completed.
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
int app_gap_le_rd_remote_feat_req_cmp_handler(ke_msg_id_t const msgid, struct gap_le_rd_remote_feat_req_cmp_evt const *param,
                                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Read remote feature: ");
        for (uint8_t i = 0; i < LE_FEATS_LEN; i++)
        {
            QPRINTF("%02x ", param->feats_used.feats[i]);
        }
        QPRINTF("\r\n");
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles read remote version information complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_RD_REM_VER_INFO_CMP_EVT
 * @param[in] param     Pointer to struct gap_rd_rem_ver_info_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the version information of remote device.
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
int app_gap_rd_remote_ver_info_cmp_handler(ke_msg_id_t const msgid, struct gap_rd_rem_ver_info_cmp_evt const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Read remote version: %d, compid: %d, subvers: %d.\r\n", param->vers, param->compid, param->subvers);
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles set random address complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_SET_RANDOM_ADDR_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_set_random_addr_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the set random address command has completed.
 ****************************************************************************************
 */
int app_gap_set_random_add_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_random_addr_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Set random addr complete.\r\n");
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles param update response from the GAP.    *//**
 *
 * @param[in] msgid     GAP_PARAM_UPDATE_RESP
 * @param[in] param     Pointer to struct gap_param_update_resp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of the connection parameter update by slave.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
int app_gap_param_update_resp_handler(ke_msg_id_t const msgid, struct gap_param_update_resp const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Slave update ");
    if (param->status == CO_ERROR_NO_ERROR && param->result == 0)
    {
        QPRINTF("success.\r\n");
    }
    else
    {
        QPRINTF("failed.\r\n");
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles update parameter complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_CHANGE_PARAM_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_change_param_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of change connection parameter.
 ****************************************************************************************
 */
int app_gap_change_param_req_cmp_handler(ke_msg_id_t const msgid, struct gap_change_param_req_cmp_evt const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Update parameter complete, interval: 0x%x, latency: 0x%x, sup to: 0x%x.\r\n", 
                                    param->con_interval, param->con_latency, param->sup_to);
    }
    else
    {
        QPRINTF("Update parameter failed.\r\n");
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles set reconnection address complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_SET_RECON_ADDR_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_set_recon_addr_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of set reconnection address.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_set_recon_addr_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_recon_addr_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Set remote reconnection addr success.\r\n");
    }
    else
    {
        QPRINTF("Set remote reconnection addr failed.\r\n");
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles set the privacy settings of the peer peripheral device complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_SET_PH_PRIVACY_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_set_ph_privacy_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of enable or disable peer privacy.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_set_ph_privacy_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_ph_privacy_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Set remote privacy flag success.\r\n");
    }
    else
    {
        QPRINTF("Set remote privacy flag failed.\r\n");
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles set privacy feature of the local device complete event from the GAP. *//**
 *
 * @param[in] msgid     GAP_SET_PRIVACY_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_set_privacy_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of enable or disable local privacy.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
int app_gap_set_privacy_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_privacy_req_cmp_evt const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("Set privacy success.\r\n");
    }
    else
    {
        QPRINTF("Set privacy failed.\r\n");
    }
    
    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles channel map update operation complete event from the GAP. *//**
 *
 * @param[in] msgid     GAP_CHANNEL_MAP_CMP_EVT
 * @param[in] param     Pointer to struct gap_channel_map_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of set or read channel map.
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
int app_gap_channel_map_cmp_handler(ke_msg_id_t const msgid, struct gap_channel_map_cmp_evt const *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Channel map request result: ");
    if (param->status == CO_ERROR_NO_ERROR)
    {
        for (uint8_t i = 0; i < LE_CHNL_MAP_LEN; i++)
        {
            QPRINTF("0x%02x ", param->chmap.map[i]);
        }
        QPRINTF("\r\n");
    }
    else
    {
        QPRINTF("failed.\r\n");
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles read RSSI value complete event from the GAP.  *//**
 *
 * @param[in] msgid     GAP_READ_RSSI_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_read_rssi_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the value of RSSI.
 ****************************************************************************************
 */
#if (QN_LINK_INFO)
int app_gap_read_rssi_req_cmp_handler(ke_msg_id_t const msgid, struct gap_read_rssi_req_cmp_evt const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    const int8_t rssi = app_correct_rssi(param->rssi);
    
    QPRINTF("Read RSSI value result: ");
    if (param->status == CO_ERROR_NO_ERROR)
    {
        QPRINTF("%d\r\n", rssi);
    }
    else
    {
        QPRINTF("failed.\r\n");
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles parameter update request indication from the slave.  *//**
 *
 * @param[in] msgid     GAP_PARAM_UPDATE_REQ_IND
 * @param[in] param     Pointer to struct gap_param_update_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of parameter update request indication from slave.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
int app_gap_param_update_req_ind_handler(ke_msg_id_t const msgid, struct gap_param_update_req_ind const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Slave request update connection parameters:\r\n");
    QPRINTF("intv_min = %#x, intv_max = %#x, latency = %#x, time_out = %#x.\r\n", param->conn_param.intv_min, 
                param->conn_param.intv_max, param->conn_param.latency, param->conn_param.time_out);
    // User may check parameters details here then to decide whether accept or reject
    if (app_check_update_conn_param((struct gap_conn_param_update *)&param->conn_param))
    {
        app_gap_change_param_req(param->conhdl, 0x0000, (struct gap_conn_param_update *)&param->conn_param);
    }
    else
    {
        app_gap_change_param_req(param->conhdl, 0x0001, (struct gap_conn_param_update *)&param->conn_param);
    }

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles bond request complete indication from the GAP.  *//**
 *
 * @param[in] msgid     GAP_BOND_REQ_CMP_EVT
 * @param[in] param     Pointer to struct gap_bond_req_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application of the outcome of bond request.
 ****************************************************************************************
 */
#if (QN_SECURITY_ON)
int app_gap_bond_req_cmp_ind_handler(ke_msg_id_t const msgid, struct gap_bond_req_cmp_evt const *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QPRINTF("Bond request complete handle: %04X, bonded: %01X, status: 0x%02X.\r\n", 
                param->conhdl,
                param->bonded,
                param->status);
    switch (param->status)
    {
    case CO_ERROR_NO_ERROR:
        if (param->bonded == 0x01)
        {
            app_env.dev_rec[param->idx].bonded_info.sec_prop = param->sec_prop;
            app_add_bonded_dev(&app_env.dev_rec[param->idx].bonded_info);
        }
        break;
    case SMP_PAIR_ERR_AUTH_REQUIREMENTS:
        // repeat BOND request by lower down
        // the security level from GAP_SEC1_AUTH_PAIR_ENC to GAP_SEC1_NOAUTH_PAIR_ENC
        // the IO capbility from SMP_IO_CAP_KB_DISPLAY to GAP_SEC1_NOAUTH_PAIR_ENC
        break;
    case SMP_PAIR_ERR_OOB_NOT_AVAILABLE:
        // repeat BOND request by change 
        // the oob from SMP_OOB_AUTH_DATA_FROM_REMOTE_DEV_PRESENT to SMP_OOB_AUTH_DATA_NOT_PRESENT
        break;
    case SMP_PAIR_ERR_ENCRYPTION_KEY_SIZE:
        // repeat BOND request by change 
        // the key size from SMPC_MIN_ENC_SIZE_LEN to SMPC_MAX_ENC_SIZE_LEN
        break;
    case SMP_PAIR_ERR_PAIRING_NOT_SUPPORTED:
        // can not repeat BOND request any more
        break;
    default:
        app_env.dev_rec[param->idx].bonded_info.sec_prop = 0;
        break;
    }
    app_env.bond_flag &= ~APP_OP_BOND;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles bond request indication from the GAP.  *//**
 *
 * @param[in] msgid     GAP_BOND_REQ_IND
 * @param[in] param     Pointer to struct gap_bond_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that remote devie wants to bond with our device.
 * The application needs to send GAP_BOND_RESP to GAP block to indicate response to the bonding request.
 ****************************************************************************************
 */
#if (!BLE_BROADCASTER && !BLE_OBSERVER)
int app_gap_bond_req_ind_handler(ke_msg_id_t const msgid, struct gap_bond_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
#if (QN_SECURITY_ON)
    uint8_t reject;
    uint8_t auth;

    // Caculate both IOP and OOB then decide whether to accept here
    reject = 0;
    /// Authentication Requirements
    switch (app_env.sec_lvl_set)
    {
    case GAP_SEC1_AUTH_PAIR_ENC:
        auth = SMP_AUTH_REQ_MITM_BOND;
        break;
    case GAP_SEC1_NOAUTH_PAIR_ENC:
        auth = SMP_AUTH_REQ_NO_MITM_BOND;
        break;
    case GAP_NO_SEC:
        auth = SMP_AUTH_REQ_MITM_NO_BOND;
        break;
    default:
        auth = SMP_AUTH_REQ_NO_MITM_BOND;
        break;
    }
    app_env.bond_flag |= APP_OP_BOND;
    // response here
    app_gap_bond_resp(app_get_conhdl_by_idx(param->index), reject, 
        SMP_OOB_AUTH_DATA_NOT_PRESENT, auth, app_env.iocap_set);
    QPRINTF("Accept Bond request indi received from %02X%02X:%02X:%02X%02X%02X.\r\n", 
        param->addr.addr[5],
        param->addr.addr[4],
        param->addr.addr[3],
        param->addr.addr[2],
        param->addr.addr[1],
        param->addr.addr[0]);
#else
    app_gap_bond_resp(app_get_conhdl_by_idx(param->index), true, 
                       SMP_OOB_AUTH_DATA_NOT_PRESENT, SMP_AUTH_REQ_NO_MITM_BOND, SMP_IO_CAP_NO_INPUT_NO_OUTPUT);
#endif

    return (KE_MSG_CONSUMED);
}
#endif

/*
 ****************************************************************************************
 * @brief Handles the GATT resource access request. *//**
 *
 * @param[in] msgid     GATT_RESOURCE_ACCESS_REQ
 * @param[in] param     Pointer to struct gatt_resource_access_req
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATT
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used to tell GATT app create database complete. When response is received
 *  by GATT, peer device is able to access database.
 *
 ****************************************************************************************
 */
int app_gatt_resource_access_req_handler(ke_msg_id_t const msgid, struct gatt_resource_access_req const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gatt_resource_access_rsp *msg = KE_MSG_ALLOC(GATT_RESOURCE_ACCESS_RSP, TASK_GATT, TASK_APP,
                                                        gatt_resource_access_rsp);

    msg->conhdl = param->conhdl;
    ke_msg_send(msg);

    return (KE_MSG_CONSUMED);
}

/// @} APP_GAP_TASK
