/**
 ****************************************************************************************
 *
 * @file app_menu.h
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

#ifndef _APP_MENU_H_
#define _APP_MENU_H_

/*
 * DEFINES
 ****************************************************************************************
 */
// menu id
typedef enum
{
    menu_start,
    menu_main,
    menu_gap,
    menu_gap_create_connection,
    menu_gap_disconnection,
    menu_gap_bond,
    menu_gap_unpair,
    menu_gatt,
    menu_gatt_disc_all_svc,
    menu_gatt_disp_all_svc,
    menu_smp,
    menu_smp_sec_lvl_set,
    menu_smp_io_cap_set,
    menu_smp_tk_input,
    menu_htpc,
    menu_htpc_enable,
    menu_htpt,
    menu_blpc,
    menu_blpc_enable,
    menu_blps,
    menu_hrpc,
    menu_hrpc_enable,
    menu_qppc,
    menu_qppc_enable,
    menu_hrpt,
    menu_glpc,
    menu_glpc_enable,
    menu_glps,
    menu_findl,
    menu_findl_enable,
    menu_findt,
    menu_proxm,
    menu_proxm_enable,
    menu_proxr,
    menu_tipc,
    menu_tipc_enable,
    menu_tips,
    menu_hogph,
    menu_hogpbh_enable,
    menu_hogprh_enable,
    menu_hogpd,
    menu_scppc,
    menu_scppc_enable,
    menu_scpps,
    menu_disc,
    menu_disc_enable,
    menu_diss,
    menu_basc,
    menu_basc_enable,
    menu_bass,
    menu_cscpc,
    menu_cscpc_enable,
    menu_cscpc_ctnl_pt,
    menu_rscpc,
    menu_rscpc_enable,
    menu_rscpc_ctnl_pt,
    menu_anpc,
    menu_anpc_enable,
    menu_anpc_alert,
    menu_paspc,
    menu_paspc_enable,
    menu_max
} MENU_ID;
 
/*
 ****************************************************************************************
 * @brief Show the BLE demo application.
 *
 ****************************************************************************************
 */
void app_menu_show(void);

/*
 ****************************************************************************************
 * @brief Handler function of the BLE demo application.
 *
 ****************************************************************************************
 */
void app_menu_hdl(void);

/*
 ****************************************************************************************
 * @brief Callback function of MENU.
 *
 ****************************************************************************************
 */
void app_menu_cb(ke_msg_id_t const msgid, void *param);

#endif // _APP_MENU_H_

