/**
 ****************************************************************************************
 *
 * @file app_menu.c
 *
 * @brief Application DEMO menu
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

static void app_menu_show_line(void)
{
	QPRINTF("*------------------------\r\n");
}

static void app_menu_show_start(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE Demo Application\r\n");
}

static void app_menu_show_main(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE Main Menu\r\n");
    QPRINTF("* 1. GAP   Menu\r\n");
#if QN_SVC_DISC_USED
    QPRINTF("* 2. GATT  Menu\r\n");
#endif
#if QN_SECURITY_ON
	QPRINTF("* 3. SMP   Menu\r\n");
#endif
#if BLE_HT_COLLECTOR
    QPRINTF("* 4. HTPC  Menu\r\n");
#endif
#if BLE_BP_COLLECTOR
    QPRINTF("* 5. BLPC  Menu\r\n");
#endif
#if BLE_HR_COLLECTOR
    QPRINTF("* 6. HRPC  Menu\r\n");
#endif
#if BLE_GL_COLLECTOR
    QPRINTF("* 7. GLPC  Menu\r\n");
#endif
#if BLE_FINDME_LOCATOR
    QPRINTF("* 8. FMPL  Menu\r\n");
#endif
#if BLE_PROX_MONITOR
    QPRINTF("* 9. PROXM Menu\r\n");
#endif
#if BLE_TIP_CLIENT
    QPRINTF("* a. TIPC  Menu\r\n");
#endif
#if (BLE_HID_BOOT_HOST || BLE_HID_REPORT_HOST)
    QPRINTF("* b. HOGH  Menu\r\n");
#endif
#if BLE_SP_CLIENT
    QPRINTF("* c. SCPPC Menu\r\n");
#endif
#if BLE_DIS_CLIENT
    QPRINTF("* d. DISC  Menu\r\n");
#endif
#if BLE_BATT_CLIENT
    QPRINTF("* e. BASC  Menu\r\n");
#endif
#if BLE_AN_CLIENT
    QPRINTF("* f. ANPC  Menu\r\n");
#endif
#if BLE_CSC_COLLECTOR
    QPRINTF("* j. CSCPC  Menu\r\n");
#endif
#if BLE_RSC_COLLECTOR
    QPRINTF("* i. RSCPC  Menu\r\n");
#endif
#if BLE_PAS_CLIENT
    QPRINTF("* g. PASPC  Menu\r\n");
#endif
#if BLE_QPP_CLIENT
    QPRINTF("* h. QPPC  Menu\r\n");
#endif
    QPRINTF("* r. Upper Menu\r\n");
    QPRINTF("* s. Show  Menu\r\n");
	app_menu_show_line();
}

static void app_menu_handler_main(void)
{
    switch (app_env.input[0])
    {
    case '1':
        app_env.menu_id = menu_gap;
        break;
#if QN_SVC_DISC_USED
    case '2':
        app_env.menu_id = menu_gatt;
        break;
#endif
#if (QN_SECURITY_ON)
	case '3':
        app_env.menu_id = menu_smp;
        break;
#endif
#if BLE_HT_COLLECTOR
    case '4':
        app_env.menu_id = menu_htpc;
        break;
#endif
#if BLE_BP_COLLECTOR
    case '5':
        app_env.menu_id = menu_blpc;
        break;
#endif
#if BLE_HR_COLLECTOR
    case '6':
        app_env.menu_id = menu_hrpc;
        break;
#endif
#if BLE_GL_COLLECTOR
    case '7':
        app_env.menu_id = menu_glpc;
        break;
#endif
#if BLE_FINDME_LOCATOR
    case '8':
        app_env.menu_id = menu_findl;
        break;
#endif
#if BLE_PROX_MONITOR
    case '9':
        app_env.menu_id = menu_proxm;
        break;
#endif
#if BLE_TIP_CLIENT
    case 'a':
        app_env.menu_id = menu_tipc;
        break;
#endif
#if (BLE_HID_BOOT_HOST || BLE_HID_REPORT_HOST)
    case 'b':
        app_env.menu_id = menu_hogph;
        break;
#endif
#if BLE_SP_CLIENT
    case 'c':
        app_env.menu_id = menu_scppc;
        break;
#endif
#if BLE_DIS_CLIENT
    case 'd':
        app_env.menu_id = menu_disc;
        break;
#endif
#if BLE_BATT_CLIENT
    case 'e':
        app_env.menu_id = menu_basc;
        break;
#endif
#if BLE_AN_CLIENT
    case 'f':
        app_env.menu_id = menu_anpc;
        break;
#endif
#if BLE_CSC_COLLECTOR
    case 'j':
        app_env.menu_id = menu_cscpc;
        break;
#endif
#if BLE_RSC_COLLECTOR
    case 'i':
        app_env.menu_id = menu_rscpc;
        break;
#endif
#if BLE_PAS_CLIENT
    case 'g':
        app_env.menu_id = menu_paspc;
        break;
#endif
#if BLE_QPP_CLIENT
    case 'h':
        app_env.menu_id = menu_qppc;
        break;
#endif
    case 'r':
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_gap(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE GAP Menu\r\n");
    QPRINTF("* 1. Inquiry Start\r\n");
    QPRINTF("* 2. Inquiry Stop\r\n");
#if (BLE_PERIPHERAL)
    QPRINTF("* 3. Advertisement Start\r\n");
    QPRINTF("* 4. Advertisement Stop\r\n");
#endif
    QPRINTF("* 5. Create Connection\r\n");
    QPRINTF("* 6. Disconnection\r\n");
#if (QN_SECURITY_ON)
    QPRINTF("* 7. Pair Request\r\n");
#endif
    QPRINTF("* 8. UnPair\r\n");
	app_menu_show_line();
}

static void app_menu_handler_gap(void)
{
    switch (app_env.input[0])
    {
    case '1':
        app_gap_dev_inq_req(GAP_GEN_INQ_TYPE, QN_ADDR_TYPE);
        break;
    case '2':
        app_gap_dev_inq_cancel_req();
        break;
#if (BLE_PERIPHERAL)
    case '3':
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE, 
            app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE), 
            app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
            GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
        break;
    case '4':
        app_gap_adv_stop_req();
        break;
#endif
    case '5':
        app_env.menu_id = menu_gap_create_connection;
        app_menu_show();
        break;
    case '6':
        app_env.menu_id = menu_gap_disconnection;
        app_menu_show();
        break;
#if (QN_SECURITY_ON)
    case '7':
        app_env.menu_id = menu_gap_bond;
        app_menu_show();
        break;
#endif
    case '8':
        app_env.menu_id = menu_gap_unpair;
        app_menu_show();
        break;
    case 'r':
        app_env.menu_id = menu_main;
    case 's':
    default:
        app_menu_show();
        break;
    }
}

static void app_menu_show_gap_inq_record(void)
{
    if (app_env.inq_idx > 0)
    {
        QPRINTF("\r\n");
        QPRINTF("Inquiry device record:\r\n");
        for (uint8_t i = 0; i < app_env.inq_idx; i++)
        {
            QPRINTF("%d. %02X%02X%02X%02X%02X%02X\r\n", i, 
                app_env.inq_addr[i].addr[5],
                app_env.inq_addr[i].addr[4],
                app_env.inq_addr[i].addr[3],
                app_env.inq_addr[i].addr[2],
                app_env.inq_addr[i].addr[1],
                app_env.inq_addr[i].addr[0]);
        }
        QPRINTF("Please input the device index:\r\n");
    }
    else
    {
        QPRINTF("Inquiry device record is NULL.\r\n");
        app_env.menu_id = menu_gap;
    }
}

static void app_menu_show_gap_dev_record(void)
{
    if (app_get_link_nb())
    {
        QPRINTF("\r\n");
        QPRINTF("Connected device records (%d):\r\n", app_get_link_nb());
        for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX; idx++)
        {
            struct bd_addr peer_addr;
            if (app_get_bd_addr_by_idx(idx, &peer_addr))
            {
                QPRINTF("%d. %02X%02X%02X%02X%02X%02X\r\n", idx, 
                    peer_addr.addr[5],
                    peer_addr.addr[4],
                    peer_addr.addr[3],
                    peer_addr.addr[2],
                    peer_addr.addr[1],
                    peer_addr.addr[0]);
            }
        }
        if (app_env.menu_id != menu_gatt_disc_all_svc
         && app_env.menu_id != menu_gap_disconnection
         && app_env.menu_id != menu_gap_bond
         && app_env.menu_id != menu_htpc_enable
         && app_env.menu_id != menu_blpc_enable
         && app_env.menu_id != menu_hrpc_enable
         && app_env.menu_id != menu_qppc_enable
         && app_env.menu_id != menu_glpc_enable
         && app_env.menu_id != menu_findl_enable
         && app_env.menu_id != menu_proxm_enable
         && app_env.menu_id != menu_tipc_enable
         && app_env.menu_id != menu_hogpbh_enable
         && app_env.menu_id != menu_hogprh_enable
         && app_env.menu_id != menu_scppc_enable
         && app_env.menu_id != menu_disc_enable
         && app_env.menu_id != menu_basc_enable
         && app_env.menu_id != menu_cscpc_enable
         && app_env.menu_id != menu_rscpc_enable
         && app_env.menu_id != menu_anpc_enable
         && app_env.menu_id != menu_paspc_enable
         )
            QPRINTF("a. All device\r\n");
        QPRINTF("Please input the device index:\r\n");
    }
    else
    {
        QPRINTF("Connected device record is NULL.\r\n");
        if (app_env.menu_id == menu_gatt_disc_all_svc)
            app_env.menu_id = menu_gatt;
        else if (app_env.menu_id == menu_qppc_enable)
            app_env.menu_id = menu_qppc;
        else if (app_env.menu_id == menu_htpc_enable)
            app_env.menu_id = menu_htpc;
        else if (app_env.menu_id == menu_blpc_enable)
            app_env.menu_id = menu_blpc;
        else if (app_env.menu_id == menu_hrpc_enable)
            app_env.menu_id = menu_hrpc;
        else if (app_env.menu_id == menu_glpc_enable)
            app_env.menu_id = menu_glpc;
        else if (app_env.menu_id == menu_findl_enable)
            app_env.menu_id = menu_findl;
        else if (app_env.menu_id == menu_proxm_enable)
            app_env.menu_id = menu_proxm;
        else if (app_env.menu_id == menu_tipc_enable)
            app_env.menu_id = menu_tipc;
        else if (app_env.menu_id == menu_hogpbh_enable)
            app_env.menu_id = menu_hogph;
        else if (app_env.menu_id == menu_hogprh_enable)
            app_env.menu_id = menu_hogph;
        else if (app_env.menu_id == menu_scppc_enable)
            app_env.menu_id = menu_scppc;
        else if (app_env.menu_id == menu_disc_enable)
            app_env.menu_id = menu_disc;
        else if (app_env.menu_id == menu_basc_enable)
            app_env.menu_id = menu_basc;
        else if (app_env.menu_id == menu_cscpc_enable)
            app_env.menu_id = menu_cscpc;
        else if (app_env.menu_id == menu_rscpc_enable)
            app_env.menu_id = menu_rscpc;
        else if (app_env.menu_id == menu_anpc_enable)
            app_env.menu_id = menu_anpc;
        else if (app_env.menu_id == menu_paspc_enable)
            app_env.menu_id = menu_paspc;
        else
            app_env.menu_id = menu_gap;
    }
}

static void app_menu_show_gap_bond_record(void)
{
#if (QN_SECURITY_ON)
    uint8_t nb = app_get_bond_nb();
#else
    uint8_t nb = 0;
#endif
    if (nb > 0)
    {
        QPRINTF("\r\n");
        QPRINTF("Bonded device records (%d):\r\n", nb);
        for (uint8_t idx = 0; idx < APP_MAX_BONDED_DEVICE_NUMBER && idx < BLE_CONNECTION_MAX; idx++)
        {
            if (app_get_bond_status(idx, SMP_KDIST_ENCKEY|SMP_KDIST_SIGNKEY|SMP_KDIST_SIGNKEY))
            {
                QPRINTF("%d. %02X%02X%02X%02X%02X%02X\r\n", idx, 
                    app_env.dev_rec[idx].bonded_info.peer_addr.addr[5],
                    app_env.dev_rec[idx].bonded_info.peer_addr.addr[4],
                    app_env.dev_rec[idx].bonded_info.peer_addr.addr[3],
                    app_env.dev_rec[idx].bonded_info.peer_addr.addr[2],
                    app_env.dev_rec[idx].bonded_info.peer_addr.addr[1],
                    app_env.dev_rec[idx].bonded_info.peer_addr.addr[0]);
            }
        }
        if (app_env.menu_id != menu_gatt_disc_all_svc
            &&
            app_env.menu_id != menu_gap_unpair)
            QPRINTF("a. All device\r\n");
        QPRINTF("Please input the device index:\r\n");
    }
    else
    {
        QPRINTF("Bonded device record is NULL.\r\n");
        app_env.menu_id = menu_gap;
    }
}

static void app_menu_handler_gap_dev_select(void)
{
    if (app_env.input[0] - '0' < BLE_CONNECTION_MAX)
    {
        app_env.select_idx = app_env.input[0] - '0';

        switch (app_env.menu_id)
        {
#if (QN_SECURITY_ON)
        case menu_gap_bond:
            {
                struct bd_addr addr;
                uint8_t auth;
                if (app_get_bd_addr_by_idx(app_env.select_idx, &addr))
                {
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
                    default:
                        auth = SMP_AUTH_REQ_NO_MITM_BOND;
                        break;
                    }
                    app_gap_bond_req(&addr, SMP_OOB_AUTH_DATA_NOT_PRESENT, auth, app_env.iocap_set);
                }
                else
                    QPRINTF("The selected device is not in connected status.\r\n");
            }
            break;
        case menu_gap_unpair:
            {
                struct bd_addr addr;
                if (app_get_bd_addr_by_idx(app_env.select_idx, &addr))
                {
                    app_gap_unpair_req(&addr, app_get_bond_nb());
                    // TODO: Deleted paired information form NVDS
                    QPRINTF("Updated link bonded info success.\r\n");
                }
                else
                    QPRINTF("The selected device is not in bonded status.\r\n");
            }
            break;
#endif
        case menu_gap_create_connection:
            if (app_get_link_nb() != BLE_CONNECTION_MAX)
            {
                app_gap_le_create_conn_req(&app_env.inq_addr[app_env.select_idx], app_env.addr_type[app_env.select_idx], QN_ADDR_TYPE, 
                                           GAP_INIT_CONN_MIN_INTV, GAP_INIT_CONN_MAX_INTV, GAP_CONN_SUPERV_TIMEOUT);
            }
            else
            {
                QPRINTF("Connection Limit Exceeded\r\n");
            }
            break;
        case menu_gap_disconnection:
            if (app_env.input[0] == 'a')
            {
                for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX; idx++)
                {
                    uint16_t conhdl = app_get_conhdl_by_idx(idx);
                    if (conhdl != 0xFFFF)
                    {
                        app_gap_discon_req(conhdl);
                    }
                }
            }
            else
            {
                uint16_t conhdl = app_get_conhdl_by_idx(app_env.select_idx);
                if (conhdl != 0xFFFF)
                    app_gap_discon_req(conhdl);
                else
                    QPRINTF("The selected device is not in connected status.\r\n");
            }
            break;
        case menu_gatt_disc_all_svc:
            {
                uint16_t conhdl = app_get_conhdl_by_idx(app_env.select_idx);
                if (conhdl != 0xFFFF)
                {
#if QN_SVC_DISC_USED
                    app_env.dev_rec[app_env.select_idx].nb_svc = 0;
                    app_gatt_disc_svc_req(GATT_DISC_ALL_SVC, conhdl);
#endif
                }
                else
                    QPRINTF("The selected device is not in connected status.\r\n");
            }
            break;
        case menu_htpc_enable:
        case menu_blpc_enable:
        case menu_hrpc_enable:
        case menu_glpc_enable:
        case menu_findl_enable:
        case menu_proxm_enable:
        case menu_tipc_enable:
        case menu_hogpbh_enable:
        case menu_hogprh_enable:
        case menu_scppc_enable:
        case menu_disc_enable:
        case menu_basc_enable:
        case menu_cscpc_enable:
        case menu_rscpc_enable:
        case menu_anpc_enable:
        case menu_paspc_enable:
        case menu_qppc_enable:
            QPRINTF("Select success.\r\n");
            break;
        default:
            break;
        }
    }
    switch (app_env.input[0])
    {
    case 'r':
    default:
        if (app_env.menu_id == menu_htpc_enable)
            app_env.menu_id = menu_htpc;
        else if (app_env.menu_id == menu_qppc_enable)
            app_env.menu_id = menu_qppc;
        else if (app_env.menu_id == menu_blpc_enable)
            app_env.menu_id = menu_blpc;
        else if (app_env.menu_id == menu_hrpc_enable)
            app_env.menu_id = menu_hrpc;
        else if (app_env.menu_id == menu_glpc_enable)
            app_env.menu_id = menu_glpc;
        else if (app_env.menu_id == menu_findl_enable)
            app_env.menu_id = menu_findl;
        else if (app_env.menu_id == menu_proxm_enable)
            app_env.menu_id = menu_proxm;
        else if (app_env.menu_id == menu_tipc_enable)
            app_env.menu_id = menu_tipc;
        else if (app_env.menu_id == menu_hogpbh_enable)
            app_env.menu_id = menu_hogph;
        else if (app_env.menu_id == menu_hogprh_enable)
            app_env.menu_id = menu_hogph;
        else if (app_env.menu_id == menu_scppc_enable)
            app_env.menu_id = menu_scppc;
        else if (app_env.menu_id == menu_disc_enable)
            app_env.menu_id = menu_disc;
        else if (app_env.menu_id == menu_basc_enable)
            app_env.menu_id = menu_basc;
        else if (app_env.menu_id == menu_cscpc_enable)
            app_env.menu_id = menu_cscpc;
        else if (app_env.menu_id == menu_rscpc_enable)
            app_env.menu_id = menu_rscpc;
        else if (app_env.menu_id == menu_anpc_enable)
            app_env.menu_id = menu_anpc;
        else if (app_env.menu_id == menu_paspc_enable)
            app_env.menu_id = menu_paspc;
        else if (app_env.menu_id == menu_gatt_disc_all_svc)
            app_env.menu_id = menu_gatt;
        else
            app_env.menu_id = menu_gap;
        break;
    case 's':
        app_menu_show();
        break;
    }
}

#if QN_SVC_DISC_USED
static void app_menu_show_gatt(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE GATT Menu\r\n");
    QPRINTF("* 1. Discovery All Services\r\n");
    QPRINTF("* 2. Display All Services\r\n");
	app_menu_show_line();
}

static void app_menu_show_gatt_disp_all_svc(void)
{
    if (app_get_link_nb())
    {
        QPRINTF("\r\n");
        QPRINTF("Service record:\r\n");
        for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX; idx++)
        {
            struct bd_addr peer_addr;
            if (app_get_bd_addr_by_idx(idx, &peer_addr))
            {
                QPRINTF("%d. %02x%02x%02x%02x%02x%02x\r\n", idx, 
                    peer_addr.addr[5],
                    peer_addr.addr[4],
                    peer_addr.addr[3],
                    peer_addr.addr[2],
                    peer_addr.addr[1],
                    peer_addr.addr[0]);
                for (uint8_t i = 0; i < app_env.dev_rec[idx].nb_svc; i++)
                {
                     QPRINTF("%04X ",app_env.dev_rec[idx].svc_rec[i].attr_hdl);
                }
                QPRINTF("\r\n");
            }
        }
    }
    else
    {
        QPRINTF("Service record is NULL.\r\n");
    }
    app_env.menu_id = menu_gatt;
}

static void app_menu_handler_gatt(void)
{
    switch (app_env.input[0])
    {
    case '1':
        app_env.menu_id = menu_gatt_disc_all_svc;
        break;
    case '2':
        app_env.menu_id = menu_gatt_disp_all_svc;
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if (QN_SECURITY_ON)
static void app_menu_show_smp(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE SMP Menu\r\n");
    QPRINTF("* 1. Security Level setting\r\n");
    QPRINTF("* Default: ");
    switch (app_env.sec_lvl_set)
    {
    case GAP_SEC1_AUTH_PAIR_ENC:
        QPRINTF("M1L3\r\n");
        break;
    case GAP_SEC1_NOAUTH_PAIR_ENC:
        QPRINTF("M1L2\r\n");
        break;
    case GAP_NO_SEC:
    default:
        QPRINTF("M1L1\r\n");
        break;
    }
    QPRINTF("* Note: Change before Adv/Scan/Connect\r\n");
    QPRINTF("* 2. IO Capbility setting\r\n");
    QPRINTF("* Default: ");
    switch (app_env.iocap_set)
    {
    case SMP_IO_CAP_KB_DISPLAY:
        QPRINTF("Keyboard Display\r\n");
        break;
    case SMP_IO_CAP_DISPLAY_ONLY:
        QPRINTF("Display Only\r\n");
        break;
    case SMP_IO_CAP_NO_INPUT_NO_OUTPUT:
    default:
        QPRINTF("No Input No Output\r\n");
        break;
    }
	app_menu_show_line();
}

static void app_menu_handler_smp(void)
{
    switch (app_env.input[0])
    {
    case '1':
        app_env.menu_id = menu_smp_sec_lvl_set;
        break;
    case '2':
        app_env.menu_id = menu_smp_io_cap_set;
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_smp_sec_lvl_set(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE Security Level\r\n");
    QPRINTF("* 0. M1L1\r\n");
    QPRINTF("* 1. M1L2\r\n");
    QPRINTF("* 2. M1L3\r\n");
    QPRINTF("* 3. M2L1\r\n");
    QPRINTF("* 4. M2L2\r\n");
	app_menu_show_line();
}

static void app_menu_handler_smp_sec_lvl_set(void)
{
    switch (app_env.input[0])
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
        app_env.sec_lvl_set = app_env.input[0] - '0';
        app_gap_set_sec_req(app_env.sec_lvl_set);
        break;
    case 'r':
        app_env.menu_id = menu_smp;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_smp_iocap_set(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE SMP IO Capability\r\n");
    QPRINTF("* 0. Display Only\r\n");
    QPRINTF("* 1. Display YesNo\r\n");
    QPRINTF("* 2. Keyboard Only\r\n");
    QPRINTF("* 3. No Input No Output\r\n");
    QPRINTF("* 4. Keyboard Display\r\n");
	app_menu_show_line();
}

static void app_menu_handler_smp_iocap_set(void)
{
    switch (app_env.input[0])
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
        app_env.iocap_set = app_env.input[0] - '0';
        break;
    case 'r':
        app_env.menu_id = menu_smp;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_smp_tk_input(void)
{
	QPRINTF("Please input the passkey:\r\n");
}

static void app_menu_handler_smp_tk_input(void)
{
    struct smp_key tk = {0};
    uint32_t i = atoi((char *)app_env.input);
    if (i <= 999999)
    {
        app_i_to_array(i, (char *)tk.key);
        app_smpc_tk_req_rsp(app_env.tk_idx, CO_ERROR_NO_ERROR, &tk);
    }
    else
    {
        app_smpc_tk_req_rsp(app_env.tk_idx, !CO_ERROR_NO_ERROR, &tk);
    }
    app_env.menu_id = menu_gap;
}
#endif

#if BLE_PAS_CLIENT
static void app_menu_show_paspc(void)
{
    app_menu_show_line();
    QPRINTF("* QN BLE PASPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Write PAS Alert Status Client Characteristic CFG. Start Notify\r\n");
    QPRINTF("* 3. Write PAS Alert Status Client Characteristic CFG. Stop Notify\r\n");
    QPRINTF("* 4. Write PAS Ringer Setting Client Characteristic CFG. Start Notify\r\n");
    QPRINTF("* 5. Write PAS PAS Ringer Setting Client Characteristic CFG. Stop Notify\r\n");
    QPRINTF("* 6. Read PAS Alert Status\r\n");
    QPRINTF("* 7. Read PAS Ringer Setting\r\n");
    QPRINTF("* 8. Write PAS Ringer Control Point\r\n");
    app_menu_show_line();
}

static void app_menu_handler_paspc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_paspc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            &&
            false == app_get_client_service_status(idx, ATT_SVC_PHONE_ALERT_STATUS))
        {
            app_paspc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable PASPC disallowed.\r\n");
        break;
    }
    case '2':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        union write_value_tag value;
        value.alert_status_ntf_cfg = PRF_CLI_START_NTF;
        app_paspc_wr_char_req(PASPC_RD_WR_ALERT_STATUS_CFG, &value, conhdl);
        break;
    }
    case '3':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        union write_value_tag value;
        value.alert_status_ntf_cfg = PRF_CLI_STOP_NTFIND;
        app_paspc_wr_char_req(PASPC_RD_WR_ALERT_STATUS_CFG, &value, conhdl);
        break;
    }
    case '4':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        union write_value_tag value;
        value.ringer_setting_ntf_cfg = PRF_CLI_START_NTF;
        app_paspc_wr_char_req(PASPC_RD_WR_RINGER_SETTING_CFG, &value, conhdl);
        break;
    }
    case '5':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        union write_value_tag value;
        value.ringer_setting_ntf_cfg = PRF_CLI_STOP_NTFIND;
        app_paspc_wr_char_req(PASPC_RD_WR_RINGER_SETTING_CFG, &value, conhdl);
        break;
    }
    case '6':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        app_paspc_read_req(PASPC_RD_ALERT_STATUS, conhdl);
        break;
    }
    case '7':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        app_paspc_read_req(PASPC_RD_RINGER_SETTING, conhdl);
        break;
    }
    case '8':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        union write_value_tag value;
        value.ringer_ctnl_pt = PASP_SILENT_MODE; // silent mode
        app_paspc_wr_char_req(PASPC_WR_RINGER_CTNL_PT, &value, conhdl);
        break;
    }
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif


#if BLE_QPP_CLIENT
static void app_menu_show_qppc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE QPPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Send Data\r\n");
	app_menu_show_line();
}

static void app_menu_handler_qppc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_qppc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(app_env.select_idx);

        if (0xFFFF != conhdl && idx < BLE_CONNECTION_MAX
            && false == app_get_qpp_client_service_status(idx))
        {
            app_qppc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable QPPC disallowed.\r\n");
        break;
    }
    
    case '2':
    {
        uint8_t test[6] = {1,2,3,4,5,6};
        app_qppc_wr_data_req(6,test,app_get_conhdl_by_idx(app_env.select_idx));
    }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_AN_CLIENT
static void app_menu_show_anpc(void)
{
    app_menu_show_line();
    QPRINTF("* QN BLE ANPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Read NEW ALERT CFG\r\n");
    QPRINTF("* 3. Read UNREAD ALERT STATUS CFG\r\n");
    QPRINTF("* 4. Write ANS New Alert Client CFG Start Notify\r\n");
    QPRINTF("* 5. Write ANS New Alert Client CFG Stop Notify\r\n");
    QPRINTF("* 6. Write ANS Unread Alert Status Client CFG Start Notify\r\n");
    QPRINTF("* 7. Write ANS Unread Alert Status Client CFG Stop Notify\r\n");
    QPRINTF("* 8. Write ANS Alert Notification Control Point\r\n");
    app_menu_show_line();
}

static void app_menu_handler_anpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_anpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            &&
            false == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            struct anp_cat_id_bit_mask new_alert_enable;
            struct anp_cat_id_bit_mask unread_alert_enable;
            new_alert_enable.cat_id_mask_0 = CAT_ID_ALL_SUPPORTED_CAT;
            new_alert_enable.cat_id_mask_1 = ANP_CAT_ID_1_MASK;
            unread_alert_enable.cat_id_mask_0 = CAT_ID_ALL_SUPPORTED_CAT;
            unread_alert_enable.cat_id_mask_1 = ANP_CAT_ID_1_MASK;
            
            app_anpc_enable_req(&new_alert_enable, &unread_alert_enable, NULL, conhdl);
        }
        else
            QPRINTF("Enable ANPC disallowed.\r\n");
        break;
    }
    case '2':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            app_anpc_rd_char_req(ANPC_RD_WR_NEW_ALERT_CFG, conhdl);
        }
        break;
    }
    case '3':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            app_anpc_rd_char_req(ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG, conhdl);
        }
        break;
    }
    case '4':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.new_alert_ntf_cfg = PRF_CLI_START_NTF; // Notify
            app_anpc_wr_char_req(ANPC_RD_WR_NEW_ALERT_CFG, &value, conhdl);
        }
        break;
    }
    case '5':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.new_alert_ntf_cfg = PRF_CLI_STOP_NTFIND; // Stop Notify
            app_anpc_wr_char_req(ANPC_RD_WR_NEW_ALERT_CFG, &value, conhdl);
        }
        break;
    }
    case '6':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.unread_alert_status_ntf_cfg = PRF_CLI_START_NTF; // Notify
            app_anpc_wr_char_req(ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG, &value, conhdl);
        }
        break;
    }
    case '7':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.unread_alert_status_ntf_cfg = PRF_CLI_STOP_NTFIND; // Stop Notify
            app_anpc_wr_char_req(ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG, &value, conhdl);
        }
        break;
    }
    case '8':
    {
        app_env.menu_id = menu_anpc_alert;
        break;
    }
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_anpc_alert(void)
{
    app_menu_show_line();
    QPRINTF("* QN BLE ANPC ALERT Menu\r\n");
    QPRINTF("* 1. Enable New Alert\r\n");
    QPRINTF("* 2. Enable New Unread Alert\r\n");
    QPRINTF("* 3. Disable All Alert\r\n");
    QPRINTF("* 4. Disable All Unread Alert\r\n");
    QPRINTF("* 5. Notify New Alert Immediately\r\n");
    QPRINTF("* 6. Notify New Unread Immediately\r\n");
    app_menu_show_line();
}

static void app_menu_handler_anpc_alert(void)
{
    uint8_t idx = app_env.select_idx;
    switch (app_env.input[0])
    {
    case '1':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;

            value.ctnl_pt.cmd_id = CMD_ID_EN_NEW_IN_ALERT_NTF;
            value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;

            app_anpc_wr_char_req(ANPC_WR_ALERT_NTF_CTNL_PT, &value, conhdl);
        }
        break;
    }
    case '2':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.ctnl_pt.cmd_id = CMD_ID_EN_UNREAD_CAT_STATUS_NTF;
            value.ctnl_pt.cat_id = CAT_ID_MISSED_CALL;
            app_anpc_wr_char_req(ANPC_WR_ALERT_NTF_CTNL_PT, &value, conhdl);
        }
        break;
    }
    case '3':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.ctnl_pt.cmd_id = CMD_ID_DIS_NEW_IN_ALERT_NTF;
            value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            app_anpc_wr_char_req(ANPC_WR_ALERT_NTF_CTNL_PT, &value, conhdl);
        }
        break;
    }
    case '4':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.ctnl_pt.cmd_id = CMD_ID_DIS_UNREAD_CAT_STATUS_NTF;
            value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            app_anpc_wr_char_req(ANPC_WR_ALERT_NTF_CTNL_PT, &value, conhdl);
        }
        break;
    }
    case '5':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.ctnl_pt.cmd_id = CMD_ID_NTF_NEW_IN_ALERT_IMM;
            value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            app_anpc_wr_char_req(ANPC_WR_ALERT_NTF_CTNL_PT, &value, conhdl);
        }
        break;
    }
    case '6':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_ALERT_NTF))
        {
            uint16_t conhdl = app_get_conhdl_by_idx(idx);
            union anpc_write_value_tag value;
            value.ctnl_pt.cmd_id = CMD_ID_NTF_UNREAD_CAT_STATUS_IMM;
            value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            app_anpc_wr_char_req(ANPC_WR_ALERT_NTF_CTNL_PT, &value, conhdl);
        }
        break;
    }
    case 'r':
        app_env.menu_id = menu_anpc;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_HT_COLLECTOR
static void app_menu_show_htpc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE HTPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Measurement Interval\r\n");
    QPRINTF("* 4. Write Measurement Interval\r\n");
    QPRINTF("* 5. Configure Temperature Measurement\r\n");
    QPRINTF("* 6. Configure Intermediate Temperature\r\n");
    QPRINTF("* 7. Configure Measurement Interval\r\n");
	app_menu_show_line();
}

static void app_menu_handler_htpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_htpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            &&
            false == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            app_htpc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable HTPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            app_htpc_cfg_indntf_req(HTPC_CHAR_HTS_TEMP_MEAS, PRF_CLI_STOP_NTFIND, app_htpc_env[idx].conhdl);
            app_htpc_cfg_indntf_req(HTPC_CHAR_HTS_INTM_TEMP, PRF_CLI_STOP_NTFIND, app_htpc_env[idx].conhdl);
            app_htpc_cfg_indntf_req(HTPC_CHAR_HTS_MEAS_INTV, PRF_CLI_STOP_NTFIND, app_htpc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            app_htpc_rd_char_req(HTPC_RD_HTS_MEAS_INTV, app_htpc_env[idx].conhdl);
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            // 10 seconds
            app_htpc_wr_meas_intv_req(10, app_htpc_env[idx].conhdl);
            app_htpc_env[idx].cur_code = 1;
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            app_htpc_cfg_indntf_req(HTPC_CHAR_HTS_TEMP_MEAS, PRF_CLI_START_IND, app_htpc_env[idx].conhdl);
            app_htpc_env[idx].cur_code = 2;
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            app_htpc_cfg_indntf_req(HTPC_CHAR_HTS_INTM_TEMP, PRF_CLI_START_NTF, app_htpc_env[idx].conhdl);
            app_htpc_env[idx].cur_code = 3;
        }
        break;
    case '7':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEALTH_THERMOM))
        {
            app_htpc_cfg_indntf_req(HTPC_CHAR_HTS_MEAS_INTV, PRF_CLI_START_IND, app_htpc_env[idx].conhdl);
            app_htpc_env[idx].cur_code = 4;
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_BP_COLLECTOR
static void app_menu_show_blpc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE BLPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Blood pressure Features\r\n");
    QPRINTF("* 4. Configure Blood pressure Measurement\r\n");
    QPRINTF("* 5. Configure Blood Intermdiate Measurement\r\n");
	app_menu_show_line();
}

static void app_menu_handler_blpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_blpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable BLPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_cfg_indntf_req(BPS_BP_MEAS_CODE, PRF_CLI_STOP_NTFIND, app_blpc_env[idx].conhdl);
            app_blpc_cfg_indntf_req(BPS_INTERM_CP_CODE, PRF_CLI_STOP_NTFIND, app_blpc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_rd_char_req(BLPC_RD_BPS_FEATURE, app_blpc_env[idx].conhdl);
            //app_blpc_env[idx].cur_code = 1;
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_cfg_indntf_req(BPS_BP_MEAS_CODE, PRF_CLI_START_IND, app_blpc_env[idx].conhdl);
            //app_blpc_env[idx].cur_code = 4;
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_cfg_indntf_req(BPS_INTERM_CP_CODE, PRF_CLI_START_NTF, app_blpc_env[idx].conhdl);
            //app_blpc_env[idx].cur_code = 5;
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_rd_char_req(BLPC_RD_BPS_BP_MEAS_CFG, app_blpc_env[idx].conhdl);
            //app_blpc_env[idx].cur_code = 2;
        }
        break;
    case '7':
        if (true == app_get_client_service_status(idx, ATT_SVC_BLOOD_PRESSURE))
        {
            app_blpc_rd_char_req(BLPC_RD_BPS_CP_MEAS_CFG, app_blpc_env[idx].conhdl);
            //app_blpc_env[idx].cur_code = 3;
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_HR_COLLECTOR
static void app_menu_show_hrpc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE HRPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Body Sensor Location\r\n");
    QPRINTF("* 4. Read Heart Rate Configure\r\n");
    QPRINTF("* 5. Configure Heart Rate Measurement\r\n");
    QPRINTF("* 6. Reset Energy\r\n");
	app_menu_show_line();
}

static void app_menu_handler_hrpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_hrpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(app_env.select_idx);

        if (0xFFFF != conhdl && idx < BLE_CONNECTION_MAX
            && false == app_get_client_service_status(idx, ATT_SVC_HEART_RATE))
        {
            app_hrpc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable HRPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEART_RATE))
        {
            app_hrpc_cfg_indntf_req(PRF_CLI_STOP_NTFIND, app_hrpc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEART_RATE))
        {
            app_hrpc_rd_char_req(HRPC_RD_HRS_BODY_SENSOR_LOC, app_hrpc_env[idx].conhdl);
            app_hrpc_env[idx].cur_code = 1;
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEART_RATE))
        {
            app_hrpc_rd_char_req(HRPC_RD_HRS_HR_MEAS_CFG, app_hrpc_env[idx].conhdl);
            app_hrpc_env[idx].cur_code = 2;
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEART_RATE))
        {
            app_hrpc_cfg_indntf_req(PRF_CLI_START_NTF, app_hrpc_env[idx].conhdl);
            app_hrpc_env[idx].cur_code = 3;
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_HEART_RATE))
        {
            app_hrpc_wr_cntl_point_req(1, app_hrpc_env[idx].conhdl);
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_GL_COLLECTOR
static void app_menu_show_glpc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE GLPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Features\r\n");
    QPRINTF("* 4. Register\r\n");
    QPRINTF("* 5. Report Number of Stored Records\r\n");
    QPRINTF("* 6. Report all Stored Records\r\n");
    QPRINTF("* 7. Abort Current Operation\r\n");
	app_menu_show_line();
}

static void app_menu_handler_glpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_glpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            app_glpc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable GLPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            app_glpc_register_req(false, app_glpc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            if (app_glpc_env[idx].op_state == OPERATION_STATE_IDLE)
            {
                app_glpc_read_features_req(app_glpc_env[idx].conhdl);
                app_glpc_env[idx].op_state = OPERATION_STATE_ONGOING;
            }
            else
                QPRINTF("Glpc is busy.\r\n");
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            // Register Notify should before RACP request 
            app_glpc_register_req(true, app_glpc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            if (app_glpc_env[idx].op_state == OPERATION_STATE_IDLE)
            {
                struct glp_racp_req req;   
                req.op_code = GLP_REQ_REP_NUM_OF_STRD_RECS;
                req.filter.operator = GLP_OP_ALL_RECS;
                req.filter.filter_type = GLP_FILTER_SEQ_NUMBER;
                req.filter.val.seq_num.min = 0;
                req.filter.val.seq_num.max = 1;
                app_glpc_racp_req(&req, app_glpc_env[idx].conhdl);
                
                app_glpc_env[idx].op_state = OPERATION_STATE_ONGOING;
            }
            else
                QPRINTF("Glpc is busy.\r\n");
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            if (app_glpc_env[idx].op_state == OPERATION_STATE_IDLE)
            {
                struct glp_racp_req req;   
                req.op_code = GLP_REQ_REP_STRD_RECS;
                req.filter.operator = GLP_OP_ALL_RECS;
                req.filter.filter_type = GLP_FILTER_SEQ_NUMBER;
                req.filter.val.seq_num.min = 0;
                req.filter.val.seq_num.max = 1;
                app_glpc_racp_req(&req, app_glpc_env[idx].conhdl);
                
                app_glpc_env[idx].op_state = OPERATION_STATE_ONGOING;
            }
            else
                QPRINTF("Glpc is busy.\r\n");
        }
        break;
    case '7':
        if (true == app_get_client_service_status(idx, ATT_SVC_GLUCOSE))
        {
            struct glp_racp_req req;
            req.op_code = GLP_REQ_ABORT_OP;
            req.filter.operator = 0;
            req.filter.filter_type = GLP_FILTER_SEQ_NUMBER;
            req.filter.val.seq_num.min = 0;
            req.filter.val.seq_num.max = 1;
            app_glpc_racp_req(&req, app_glpc_env[idx].conhdl);
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_FINDME_LOCATOR
static void app_menu_show_findl(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE FMPL Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Alert None\r\n");
    QPRINTF("* 3. Alert Mild\r\n");
    QPRINTF("* 4. Alert High\r\n");
	app_menu_show_line();
}

static void app_menu_handler_findl(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_findl_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(app_env.select_idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_IMMEDIATE_ALERT))
            app_findl_enable_req(NULL, conhdl);
        else
            QPRINTF("Enable FMPL disallowed.\r\n");
        break;
    }
    case '2':
    case '3':
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_IMMEDIATE_ALERT))
            app_findl_set_alert_req(app_env.input[0]-'2', app_findl_env[idx].conhdl);
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_PROX_MONITOR
static void app_menu_show_proxm(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE PROXM Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Set Link Loss None\r\n");
    QPRINTF("* 3. Set Link Loss Mild\r\n");
    QPRINTF("* 4. Set Link Loss High\r\n");
    QPRINTF("* 5. Set Immediate Alert None\r\n");
    QPRINTF("* 6. Set Immediate Alert Mild\r\n");
    QPRINTF("* 7. Set Immediate Alert High\r\n");
    QPRINTF("* 8. Read LLS Level\r\n");
    QPRINTF("* 9. Read TX Power\r\n");
	app_menu_show_line();
}

static void app_menu_handler_proxm(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_proxm_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(app_env.select_idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_LINK_LOSS))
        {
            app_proxm_enable_req(NULL, NULL, NULL, conhdl);
        }
        else
            QPRINTF("Enable PROXM disallowed.\r\n");
        break;
    }
    case '2':
    case '3':
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_LINK_LOSS))
            app_proxm_wr_alert_lvl_req(PROXM_SET_LK_LOSS_ALERT, app_env.input[0]-'2', app_proxm_env[idx].conhdl);
        break;
    case '5':
    case '6':
    case '7':
        if (true == app_get_client_service_status(idx, ATT_SVC_LINK_LOSS))
            app_proxm_wr_alert_lvl_req(PROXM_SET_IMMDT_ALERT, app_env.input[0]-'5', app_proxm_env[idx].conhdl);
        break;
    case '8':
        if (true == app_get_client_service_status(idx, ATT_SVC_LINK_LOSS))
        {
            app_proxm_rd_alert_lvl_req(app_proxm_env[idx].conhdl);
            app_proxm_env[idx].cur_code = 1;
        }
        break;
    case '9':
        if (true == app_get_client_service_status(idx, ATT_SVC_LINK_LOSS))
        {
            app_proxm_rd_txpw_lvl_req(app_proxm_env[idx].conhdl);
            app_proxm_env[idx].cur_code = 0;
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_TIP_CLIENT
static void app_menu_show_tipc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE TIPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Current Time\r\n");
    QPRINTF("* 4. Configure Current Time\r\n");
    QPRINTF("* 5. Read Local Time Info\r\n");
    QPRINTF("* 6. Read Reference Time Infor\r\n");
    QPRINTF("* 7. Writing Time Control Point\r\n");
    QPRINTF("* 8. Read Time With DST\r\n");
    QPRINTF("* 9. Read Time Update State\r\n");
	app_menu_show_line();
}

static void app_menu_handler_tipc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_tipc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl && idx < BLE_CONNECTION_MAX
            && false == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            app_tipc_enable_req(NULL, NULL, NULL, conhdl);
        }
        else
            QPRINTF("Enable TIPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            app_tipc_ct_ntf_cfg_req(PRF_CLI_STOP_NTFIND, app_tipc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Read CTS Current Time
            app_tipc_rd_char_req(TIPC_RD_CTS_CURR_TIME, app_tipc_env[idx].conhdl);
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Configuring the Current Time Characteristic on the Server
            app_tipc_ct_ntf_cfg_req(PRF_CLI_START_NTF, app_tipc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Read CTS Local Time Info
            app_tipc_rd_char_req(TIPC_RD_CTS_LOCAL_TIME_INFO, app_tipc_env[idx].conhdl);
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Read CTS Reference Time Info
            app_tipc_rd_char_req(TIPC_RD_CTS_REF_TIME_INFO, app_tipc_env[idx].conhdl);
        }
        break;
    case '7':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Writing the Time Update Control Point on the Server
            app_tipc_wr_time_udp_ctnl_pt_req(TIPS_TIME_UPD_CTNL_PT_GET, app_tipc_env[idx].conhdl);
        }
        break;
    case '8':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Read RTUS Time With DST
            app_tipc_rd_char_req(TIPC_RD_NDCS_TIME_WITH_DST, app_tipc_env[idx].conhdl);
        }
        break;
    case '9':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Read RTUS Time Update State
            app_tipc_rd_char_req(TIPC_RD_RTUS_TIME_UPD_STATE, app_tipc_env[idx].conhdl);
        }
        break;
#if 0
    case 'a':
        if (true == app_get_client_service_status(idx, ATT_SVC_CURRENT_TIME))
        {
            ///Read CTS Current Time Client Cfg. Desc
            app_tipc_rd_char_req(TIPC_RD_CTS_CURR_TIME_CLI_CFG, app_tipc_env[idx].conhdl);
        }
        break;
#endif
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if (BLE_HID_BOOT_HOST || BLE_HID_REPORT_HOST)
static void app_menu_show_hogph(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE HOGH Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
#if BLE_HID_BOOT_HOST
    QPRINTF("* 1. Enable Boot Mode\r\n");
    QPRINTF("* 2. Disable Boot Mode\r\n");
#endif    
#if BLE_HID_REPORT_HOST
    QPRINTF("* 3. Enable Report Mode\r\n");
    QPRINTF("* 4. Disable Report Mode\r\n");
#endif    
	app_menu_show_line();
}

static void app_menu_handler_hogph(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_hogpbh_enable;
        break;
    case '1':
    {
#if BLE_HID_BOOT_HOST
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_HID))
        {
            app_hogpbh_enable_req(0, NULL, conhdl);
        }        
        else
            QPRINTF("Enable HID Boot Host disallowed.\r\n");
        break;
#endif
    }
    case '2':
#if BLE_HID_BOOT_HOST
        if (true == app_get_client_service_status(idx, ATT_SVC_HID))
        {               
            if (app_hogpbh_env[idx].hids_kb < HOGPBH_NB_HIDS_INST_MAX)
                app_hogpbh_cfg_ntf_req(HOGPBH_DESC_BOOT_KB_IN_REPORT_CFG, PRF_CLI_STOP_NTFIND, 
                    app_hogpbh_env[idx].hids_kb, app_hogpbh_env[idx].conhdl);
            if (app_hogpbh_env[idx].hids_mouse < HOGPBH_NB_HIDS_INST_MAX)
                app_hogpbh_cfg_ntf_req(HOGPBH_DESC_BOOT_MOUSE_IN_REPORT_CFG, PRF_CLI_STOP_NTFIND, 
                    app_hogpbh_env[idx].hids_mouse, app_hogpbh_env[idx].conhdl);
            app_hogpbh_disable_req(app_hogpbh_env[idx].conhdl);
        }
#endif
        break;
    case '3':
    {
#if BLE_HID_REPORT_HOST
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_HID))
        {
            app_hogprh_enable_req(0, NULL, conhdl);
        }        
        else
            QPRINTF("Enable HID Report Host disallowed.\r\n");
#endif
        break;
    }
    case '4':
#if BLE_HID_REPORT_HOST
        if (true == app_get_client_service_status(idx, ATT_SVC_HID))
        {               
            app_hogprh_cfg_ntf_req(0, PRF_CLI_STOP_NTFIND, 0, app_hogprh_env[idx].conhdl);
            app_hogprh_disable_req(app_hogprh_env[idx].conhdl);
        }
#endif
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_SP_CLIENT
static void app_menu_show_scppc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE SCPPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Scan Refresh Cfg\r\n");
    QPRINTF("* 4. Send Scan Interval and Win\r\n");
    QPRINTF("* 5. Configure Scan Refresh\r\n");
	app_menu_show_line();
}

static void app_menu_handler_scppc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_scppc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_SCAN_PARAMETERS))
        {
            app_scppc_enable_req(0x0012, 0x0012, NULL, conhdl);
        }
        else
            QPRINTF("Enable SCPPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_SCAN_PARAMETERS))
        {
            app_scppc_wr_meas_intv_req(PRF_CLI_STOP_NTFIND, app_scppc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_SCAN_PARAMETERS))
        {
            app_scppc_scan_refresh_ntf_cfg_rd_req(app_scppc_env[idx].conhdl);
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_SCAN_PARAMETERS))
        {
            app_scppc_scan_intv_wd_wr_req(GAP_SCAN_FAST_INTV, GAP_SCAN_FAST_WIND, app_scppc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_SCAN_PARAMETERS))
        {
            app_scppc_wr_meas_intv_req(PRF_CLI_START_NTF, app_scppc_env[idx].conhdl);
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_DIS_CLIENT
static void app_menu_show_disc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE DISC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Read Manufacturer name\r\n");
    QPRINTF("* 3. Read Model Number\r\n");
    QPRINTF("* 4. Read Serial Number\r\n");
    QPRINTF("* 5. Read Hardware Revision\r\n");
    QPRINTF("* 6. Read Firmware Revision\r\n");
    QPRINTF("* 7. Read Software Revision\r\n");
    QPRINTF("* 8. Read System ID\r\n");
    QPRINTF("* 9. Read IEEE\r\n");
    QPRINTF("* a. Read PnP ID\r\n");
	app_menu_show_line();
}

static void app_menu_handler_disc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_disc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);
        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable DISC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_MANUFACTURER_NAME_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_MODEL_NB_STR_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_SERIAL_NB_STR_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_HARD_REV_STR_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_FIRM_REV_STR_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '7':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_SW_REV_STR_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '8':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_SYSTEM_ID_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case '9':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_IEEE_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case 'a':
        if (true == app_get_client_service_status(idx, ATT_SVC_DEVICE_INFO))
        {
            app_disc_rd_char_req(DISC_PNP_ID_CHAR, app_disc_env[idx].conhdl);
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_BATT_CLIENT
static void app_menu_show_basc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE BASC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read Battery Level Format\r\n");
    QPRINTF("* 4. Read Battery Level\r\n");
    QPRINTF("* 5. Confiure Battery Level\r\n");
    QPRINTF("* 6. Read Battery Level Cfg\r\n");
	app_menu_show_line();
}

static void app_menu_handler_basc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_basc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl && idx < BLE_CONNECTION_MAX
            && false == app_get_client_service_status(idx, ATT_SVC_BATTERY_SERVICE))
        {
            app_basc_enable_req(0, NULL, conhdl);
        }        
        else
            QPRINTF("Enable BASC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_BATTERY_SERVICE))
        {
            app_basc_cfg_indntf_req(PRF_CLI_STOP_NTFIND, 0, app_basc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_BATTERY_SERVICE))
        {
            app_basc_rd_char_req(BASC_RD_BAS_BATT_LEVEL_PRES_FORMAT, 0, app_basc_env[idx].conhdl);
            //app_basc_env[idx].cur_code = 1;
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_BATTERY_SERVICE))
        {
            app_basc_rd_char_req(BASC_RD_BAS_BATT_LEVEL, 0, app_basc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_BATTERY_SERVICE))
        {
            app_basc_cfg_indntf_req(PRF_CLI_START_NTF, 0, app_basc_env[idx].conhdl);
            //app_basc_env[idx].cur_code = 0;
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_BATTERY_SERVICE))
        {
            app_basc_rd_char_req(BASC_RD_BAS_BATT_LEVEL_CLI_CFG, 0, app_basc_env[idx].conhdl);
            //app_basc_env[idx].cur_code = 2;
        }
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}
#endif

#if BLE_CSC_COLLECTOR
static void app_menu_show_cscpc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE CSCPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read CSC Feature\r\n");
    QPRINTF("* 4. Read Sensor Location\r\n");
    QPRINTF("* 5. Configure CSC Measurement\r\n");
    QPRINTF("* 6. Configure SC Control Point\r\n");
    QPRINTF("* 7. SC Control Point\r\n");
	app_menu_show_line();
}

static void app_menu_handler_cscpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_cscpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            app_cscpc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable CSCPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            app_cscpc_cfg_ntfind_req(CSCPC_RD_WR_CSC_MEAS_CFG, PRF_CLI_STOP_NTFIND, app_cscpc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            app_cscpc_read_req(CSCPC_RD_CSC_FEAT, app_cscpc_env[idx].conhdl);
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            app_cscpc_read_req(CSCPC_RD_SENSOR_LOC, app_cscpc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            app_cscpc_cfg_ntfind_req(CSCPC_RD_WR_CSC_MEAS_CFG, PRF_CLI_START_NTF, app_cscpc_env[idx].conhdl);
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            app_cscpc_cfg_ntfind_req(CSCPC_RD_WR_SC_CTNL_PT_CFG, PRF_CLI_START_IND, app_cscpc_env[idx].conhdl);
        }
        break;
    case '7':
        app_env.menu_id = menu_cscpc_ctnl_pt;
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_cscpc_ctnl_pt(void)
{
    app_menu_show_line();
    QPRINTF("* QN BLE CSCPC Conrol Menu\r\n");
    QPRINTF("* 1. Set Cumulative Value\r\n");
    QPRINTF("* 2. Update Sensor Location\r\n");
    QPRINTF("* 3. Request Supported Sensor Locations\r\n");
    app_menu_show_line();
}

static void app_menu_handler_cscpc_ctnl_pt(void)
{
    uint8_t idx = app_env.select_idx;
    switch (app_env.input[0])
    {
    case '1':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            struct cscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = CSCP_CTNL_PT_OP_SET_CUMUL_VAL;
            sc_ctnl_pt.value.cumul_val = 0;
            app_cscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_cscpc_env[idx].conhdl);
        }
        break;
    }
    case '2':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            struct cscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = CSCP_CTNL_PT_OP_UPD_LOC;
            sc_ctnl_pt.value.sensor_loc = CSCP_LOC_OTHER;
            app_cscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_cscpc_env[idx].conhdl);
        }
        break;
    }
    case '3':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_CYCLING_SPEED_CADENCE))
        {
            struct cscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = CSCP_CTNL_PT_OP_REQ_SUPP_LOC;
            app_cscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_cscpc_env[idx].conhdl);
        }
        break;
    }
    case 'r':
        app_env.menu_id = menu_cscpc;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

#endif

#if BLE_RSC_COLLECTOR
static void app_menu_show_rscpc(void)
{
	app_menu_show_line();
	QPRINTF("* QN BLE RSCPC Menu\r\n");
    QPRINTF("* 0. Select device\r\n");
    QPRINTF("* 1. Enable\r\n");
    QPRINTF("* 2. Disable\r\n");
    QPRINTF("* 3. Read RSC Feature\r\n");
    QPRINTF("* 4. Read Sensor Location\r\n");
    QPRINTF("* 5. Confiure RSC Measurement\r\n");
    QPRINTF("* 6. Confiure SC Control Point\r\n");
    QPRINTF("* 7. SC Control Point\r\n");
	app_menu_show_line();
}

static void app_menu_handler_rscpc(void)
{
    uint8_t idx = app_env.select_idx;

    switch (app_env.input[0])
    {
    case '0':
        app_env.menu_id = menu_rscpc_enable;
        break;
    case '1':
    {
        uint16_t conhdl = app_get_conhdl_by_idx(idx);

        if (0xFFFF != conhdl
            && false == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            app_rscpc_enable_req(NULL, conhdl);
        }
        else
            QPRINTF("Enable RSCPC disallowed.\r\n");
        break;
    }
    case '2':
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            app_rscpc_cfg_ntfind_req(RSCPC_RD_WR_RSC_MEAS_CFG, PRF_CLI_STOP_NTFIND, app_rscpc_env[idx].conhdl);
        }
        break;
    case '3':
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            app_rscpc_read_req(RSCPC_RD_RSC_FEAT, app_rscpc_env[idx].conhdl);
        }
        break;
    case '4':
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            app_rscpc_read_req(RSCPC_RD_SENSOR_LOC, app_rscpc_env[idx].conhdl);
        }
        break;
    case '5':
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            app_rscpc_cfg_ntfind_req(RSCPC_RD_WR_RSC_MEAS_CFG, PRF_CLI_START_NTF, app_rscpc_env[idx].conhdl);
        }
        break;
    case '6':
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            app_rscpc_cfg_ntfind_req(RSCPC_RD_WR_SC_CTNL_PT_CFG, PRF_CLI_START_IND, app_rscpc_env[idx].conhdl);
        }
        break;
    case '7':
        app_env.menu_id = menu_rscpc_ctnl_pt;
        break;
    case 'r':
        app_env.menu_id = menu_main;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

static void app_menu_show_rscpc_ctnl_pt(void)
{
    app_menu_show_line();
    QPRINTF("* QN BLE RSCPC Conrol Menu\r\n");
    QPRINTF("* 1. Set Cumulative Value\r\n");
    QPRINTF("* 2. Start Sensor Calibration\r\n");
    QPRINTF("* 3. Update Sensor Location\r\n");
    QPRINTF("* 4. Request Supported Sensor Locations\r\n");
    app_menu_show_line();
}

static void app_menu_handler_rscpc_ctnl_pt(void)
{
    uint8_t idx = app_env.select_idx;
    switch (app_env.input[0])
    {
    case '1':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            struct rscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_SET_CUMUL_VAL;
            sc_ctnl_pt.value.cumul_val = 0;
            app_rscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_rscpc_env[idx].conhdl);
        }
        break;
    }
    case '2':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            struct rscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_START_CALIB;
            app_rscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_rscpc_env[idx].conhdl);
        }
        break;
    }
    case '3':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            struct rscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_UPD_LOC;
            sc_ctnl_pt.value.sensor_loc = RSCP_LOC_OTHER;
            app_rscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_rscpc_env[idx].conhdl);
        }
        break;
    }
    case '4':
    {
        if (true == app_get_client_service_status(idx, ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            struct rscp_sc_ctnl_pt_req sc_ctnl_pt;

            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_REQ_SUPP_LOC;
            app_rscpc_ctnl_pt_cfg_req(&sc_ctnl_pt, app_rscpc_env[idx].conhdl);
        }
        break;
    }
    case 'r':
        app_env.menu_id = menu_rscpc;
        break;
    case 's':
    default:
        break;
    }
    app_menu_show();
}

#endif

/**
 ****************************************************************************************
 * @brief Show the BLE demo application.
 *
 ****************************************************************************************
 */
void app_menu_show(void)
{
    switch (app_env.menu_id)
    {
    case menu_start:
        app_menu_show_start();
        break;
    case menu_main:
        app_menu_show_main();
        break;
    case menu_gap:
        app_menu_show_gap();
        break;
#if QN_SVC_DISC_USED
	case menu_gatt:
        app_menu_show_gatt();
        break;
    case menu_gatt_disp_all_svc:
        app_menu_show_gatt_disp_all_svc();
        break;
#endif
    case menu_gap_create_connection:
        app_menu_show_gap_inq_record();
        break;
    case menu_gap_unpair:
        app_menu_show_gap_bond_record();
        break;
    case menu_gap_bond:
    case menu_gap_disconnection:
    case menu_gatt_disc_all_svc:
    case menu_htpc_enable:
    case menu_blpc_enable:
    case menu_hrpc_enable:
    case menu_glpc_enable:
    case menu_findl_enable:
    case menu_tipc_enable:
    case menu_proxm_enable:
    case menu_hogpbh_enable:
    case menu_hogprh_enable:
    case menu_scppc_enable:
    case menu_disc_enable:
    case menu_basc_enable:
    case menu_cscpc_enable:
    case menu_rscpc_enable:
    case menu_anpc_enable:
    case menu_paspc_enable:
    case menu_qppc_enable:
        app_menu_show_gap_dev_record();
        break;
#if QN_SECURITY_ON
    case menu_smp:
        app_menu_show_smp();
        break;
    case menu_smp_sec_lvl_set:
        app_menu_show_smp_sec_lvl_set();
        break;
    case menu_smp_io_cap_set:
        app_menu_show_smp_iocap_set();
        break;
    case menu_smp_tk_input:
        app_menu_show_smp_tk_input();
        break;
#endif

#if BLE_PAS_CLIENT
    case menu_paspc:
        app_menu_show_paspc();
        break;
#endif
#if BLE_AN_CLIENT
    case menu_anpc:
        app_menu_show_anpc();
        break;
    case menu_anpc_alert:
        app_menu_show_anpc_alert();
        break;
#endif
#if BLE_HT_COLLECTOR
    case menu_htpc:
        app_menu_show_htpc();
        break;
#endif
#if BLE_BP_COLLECTOR
    case menu_blpc:
        app_menu_show_blpc();
        break;
#endif
#if BLE_HR_COLLECTOR
    case menu_hrpc:
        app_menu_show_hrpc();
        break;
#endif
#if BLE_GL_COLLECTOR
    case menu_glpc:
        app_menu_show_glpc();
        break;
#endif
#if BLE_FINDME_LOCATOR
    case menu_findl:
        app_menu_show_findl();
        break;
#endif
#if BLE_PROX_MONITOR
    case menu_proxm:
        app_menu_show_proxm();
        break;
#endif
#if BLE_TIP_CLIENT
    case menu_tipc:
        app_menu_show_tipc();
        break;
#endif
#if (BLE_HID_BOOT_HOST || BLE_HID_REPORT_HOST)
    case menu_hogph:
        app_menu_show_hogph();
        break;
#endif
#if BLE_SP_CLIENT
    case menu_scppc:
        app_menu_show_scppc();
        break;
#endif
#if BLE_DIS_CLIENT
    case menu_disc:
        app_menu_show_disc();
        break;
#endif
#if BLE_BATT_CLIENT
    case menu_basc:
        app_menu_show_basc();
        break;
#endif
#if BLE_CSC_COLLECTOR
    case menu_cscpc:
        app_menu_show_cscpc();
        break;
    case menu_cscpc_ctnl_pt:
        app_menu_show_cscpc_ctnl_pt();
        break;
#endif
#if BLE_RSC_COLLECTOR
    case menu_rscpc:
        app_menu_show_rscpc();
        break;
    case menu_rscpc_ctnl_pt:
        app_menu_show_rscpc_ctnl_pt();
        break;
#endif
#if BLE_QPP_CLIENT
    case menu_qppc:
        app_menu_show_qppc();
        break;
#endif
    case menu_htpt:
    case menu_blps:
    case menu_hrpt:
    case menu_glps:
    case menu_findt:
    case menu_proxr:
    case menu_tips:
    case menu_hogpd:
    case menu_scpps:
    case menu_diss:
    case menu_bass:
    default:
        break;
    }
}

/**
 ****************************************************************************************
 * @brief Handler function of the BLE demo application.
 *
 ****************************************************************************************
 */
void app_menu_hdl(void)
{
    switch (app_env.menu_id)
    {
    case menu_start:
        break;
    case menu_main:
        app_menu_handler_main();
        break;
    case menu_gap:
        app_menu_handler_gap();
        break;
#if QN_SVC_DISC_USED
    case menu_gatt:
        app_menu_handler_gatt();
        break;
#endif		
#if QN_SECURITY_ON
	case menu_smp:
        app_menu_handler_smp();
        break;
#endif
#if BLE_QPP_CLIENT
    case menu_qppc:
        app_menu_handler_qppc();
        break;
#endif
#if BLE_PAS_CLIENT
    case menu_paspc:
        app_menu_handler_paspc();
        break;
#endif
#if BLE_AN_CLIENT
    case menu_anpc:
        app_menu_handler_anpc();
        break;
    case menu_anpc_alert:
        app_menu_handler_anpc_alert();
        break;
#endif
#if BLE_HT_COLLECTOR
    case menu_htpc:
        app_menu_handler_htpc();
        break;
#endif
#if BLE_BP_COLLECTOR
    case menu_blpc:
        app_menu_handler_blpc();
        break;
#endif
#if BLE_HR_COLLECTOR
    case menu_hrpc:
        app_menu_handler_hrpc();
        break;
#endif
#if BLE_GL_COLLECTOR
    case menu_glpc:
        app_menu_handler_glpc();
        break;
#endif
#if BLE_FINDME_LOCATOR
    case menu_findl:
        app_menu_handler_findl();
        break;
#endif
#if BLE_PROX_MONITOR
    case menu_proxm:
        app_menu_handler_proxm();
        break;
#endif
#if BLE_TIP_CLIENT
    case menu_tipc:
        app_menu_handler_tipc();
        break;
#endif
#if (BLE_HID_BOOT_HOST || BLE_HID_REPORT_HOST)
    case menu_hogph:
        app_menu_handler_hogph();
        break;
#endif
#if BLE_SP_CLIENT
    case menu_scppc:
        app_menu_handler_scppc();
        break;
#endif
#if BLE_DIS_CLIENT
    case menu_disc:
        app_menu_handler_disc();
        break;
#endif
#if BLE_BATT_CLIENT
    case menu_basc:
        app_menu_handler_basc();
        break;
#endif
#if BLE_CSC_COLLECTOR
    case menu_cscpc:
        app_menu_handler_cscpc();
        break;
    case menu_cscpc_ctnl_pt:
        app_menu_handler_cscpc_ctnl_pt();
        break;
#endif
#if BLE_RSC_COLLECTOR
    case menu_rscpc:
        app_menu_handler_rscpc();
        break;
    case menu_rscpc_ctnl_pt:
        app_menu_handler_rscpc_ctnl_pt();
        break;
#endif
    case menu_gap_bond:
    case menu_gap_unpair:
    case menu_gap_create_connection:
    case menu_gap_disconnection:

#if QN_SVC_DISC_USED
    case menu_gatt_disc_all_svc:
#endif

    case menu_htpc_enable:
    case menu_blpc_enable:
    case menu_hrpc_enable:
    case menu_glpc_enable:
    case menu_findl_enable:
    case menu_proxm_enable:
    case menu_tipc_enable:
    case menu_hogpbh_enable:
    case menu_hogprh_enable:
    case menu_scppc_enable:
    case menu_disc_enable:
    case menu_basc_enable:
    case menu_cscpc_enable:
    case menu_rscpc_enable:
    case menu_anpc_enable:
    case menu_paspc_enable:
    case menu_qppc_enable:
        app_menu_handler_gap_dev_select();
        break;
#if (QN_SECURITY_ON)
    case menu_smp_sec_lvl_set:
        app_menu_handler_smp_sec_lvl_set();
        break;
    case menu_smp_io_cap_set:
        app_menu_handler_smp_iocap_set();
        break;
     case menu_smp_tk_input:
        app_menu_handler_smp_tk_input();
        break;
#endif
    case menu_htpt:
    case menu_blps:
    case menu_hrpt:
    case menu_glps:
    case menu_findt:
    case menu_proxr:
    case menu_tips:
    case menu_hogpd:
    case menu_scpps:
    case menu_diss:
    case menu_bass:
    default:
        break;
    }
}

/**
 ****************************************************************************************
 * @brief Callback function of MENU.
 *
 ****************************************************************************************
 */
void app_menu_cb(ke_msg_id_t const msgid, void *param)
{
	ke_state_t state = (ke_state_get(TASK_APP));
	
    if (APP_INIT == state)
    {
	    switch (msgid)
	    {
        case GAP_READY_EVT:
	    case GAP_RESET_REQ_CMP_EVT:
			app_env.menu_id = menu_start;
			app_menu_show();
	        break;
		case GAP_READ_VER_REQ_CMP_EVT:
	        QPRINTF("*\r\n");
	        QPRINTF("* Version:\r\n");
	        QPRINTF("*   AP%sHL%1x.%1x.%1xLL%1x.%1x.%1x\r\n",
	            QN_APP_VER,
	            ((struct gap_read_ver_req_cmp_evt *)param)->host_ver,
	            ((struct gap_read_ver_req_cmp_evt *)param)->host_subver>>8&0xFF,
	            ((struct gap_read_ver_req_cmp_evt *)param)->host_subver&0xFF,
	            ((struct gap_read_ver_req_cmp_evt *)param)->lmp_ver,
	            ((struct gap_read_ver_req_cmp_evt *)param)->lmp_subver>>8&0xFF,
	            ((struct gap_read_ver_req_cmp_evt *)param)->lmp_subver&0xFF);
			break;
		case GAP_READ_BDADDR_REQ_CMP_EVT:
			QPRINTF("* BD Address:\r\n");
			QPRINTF("*          %02x%02x%02x%02x%02x%02x\r\n",
				((struct gap_read_bdaddr_req_cmp_evt *)param)->addr.addr[5],
				((struct gap_read_bdaddr_req_cmp_evt *)param)->addr.addr[4],
				((struct gap_read_bdaddr_req_cmp_evt *)param)->addr.addr[3],
				((struct gap_read_bdaddr_req_cmp_evt *)param)->addr.addr[2],
				((struct gap_read_bdaddr_req_cmp_evt *)param)->addr.addr[1],
				((struct gap_read_bdaddr_req_cmp_evt *)param)->addr.addr[0]);
			QPRINTF("*\r\n");
			QPRINTF("* By Derek\r\n");
			QPRINTF("* @ %11s, %8s\r\n",
				__DATE__, 
				__TIME__);
			QPRINTF("*------------------------\r\n");		
			app_env.menu_id = menu_main;
			app_menu_show();
			break;
			
		default:
			break;
	    }
    }
    else
	if (APP_IDLE == state)
    {
	    switch (msgid)
	    {
#if (QN_SECURITY_ON)
	    case SMPC_TK_REQ_IND:
			if (app_env.tk_type != 0)
			{	   
				app_env.tk_idx = ((struct smpc_tk_req_ind *)param)->idx;
				app_env.menu_id = menu_smp_tk_input;
				app_menu_show();
			}
	        break;
#endif
		default:
			break;
	    }
    }
}
#endif
/// @} APP
