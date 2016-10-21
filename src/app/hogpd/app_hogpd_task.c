/**
 ****************************************************************************************
 *
 * @file app_hogpd_task.c
 *
 * @brief Application HOGPD implementation
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
 * @addtogroup APP_HOGPD_TASK
 * @
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HID_DEVICE
#include "app_hogpd.h"

/// @cond

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_hogpd_env_tag *app_hogpd_env = &app_env.hogpd_ev;

//DEVELOPER NOTE: SET report_map to REAL VALUE OF USER APPLICATION HERE
static const uint8_t report_map[] =
{
    0x05, 0x01,
    0x09, 0x02,
    0xA1, 0x01,
    0x09, 0x01,
    0xA1, 0x00,
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x06,
    0xC0,
    0x05, 0x09,
    0x19, 0x01,
    0x29, 0x03,
    0x15, 0x00,
    0x25, 0x01,
    0x95, 0x03,
    0x75, 0x01,
    0x81, 0x02,
    0x95, 0x01,
    0x75, 0x05,
    0x81, 0x03,
    0xC0,
};

//DEVELOPER NOTE: SET boot_kb_in_report to REAL VALUE OF USER APPLICATION HERE
uint8_t boot_kb_in_report[] = {0xab, 0xcd, 0xef, 0xab, 0xcd, 0xef, 0xab, 0xcd};

//DEVELOPER NOTE: SET boot_mouse_in_report to REAL VALUE OF USER APPLICATION HERE
uint8_t boot_mouse_in_report[] = {0x01, 0x02, 0x03, 0x04};

//DEVELOPER NOTE: SET in_report to REAL VALUE OF USER APPLICATION HERE
uint8_t in_report[] = {0x0a, 0x0b, 0x0c, 0x0d};


/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the HOGPD.      *//**
 *
 * @param[in] msgid     HOGPD_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct hogpd_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation. It contains status of database creation.
 ****************************************************************************************
 */
int app_hogpd_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct hogpd_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_HID_DEVICE_BIT);
        //app_init_local_smp_key();
        app_hogpd_report_map_req(sizeof(report_map), 0, (uint8_t *)report_map);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the HOGPD disable indication message from the HOGPD.    *//**
 *
 * @param[in] msgid     HOGPD_DISABLE_IND
 * @param[in] param     Pointer to the struct hogpd_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. 
 ****************************************************************************************
 */
int app_hogpd_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct hogpd_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_hogpd_env->conhdl = 0xFFFF;
    app_hogpd_env->enabled = false;
    app_hogpd_env->ntf_sending = 0;
    ke_timer_clear(APP_HOGPD_BOOT_KB_IN_REPORT_TIMER, TASK_APP);
    ke_timer_clear(APP_HOGPD_BOOT_MOUSE_IN_REPORT_TIMER, TASK_APP);
    ke_timer_clear(APP_HOGPD_REPORT_TIMER, TASK_APP);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the HOGPD.      *//**
 *
 * @param[in] msgid     HOGPD_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called if an error has been raised during the communication.
 ****************************************************************************************
 */
int app_hogpd_error_ind_handler(ke_msg_id_t const msgid,
                                struct prf_server_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the HOGPD.    *//**
 *
 * @param[in] msgid     HOGPD_NTF_CFG_IND
 * @param[in] param     Pointer to the struct hogpd_ntf_cfg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 *
 ****************************************************************************************
 */
int app_hogpd_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                  struct hogpd_ntf_cfg_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    switch (param->cfg_code)
    {
        case HOGPD_BOOT_KB_IN_REPORT_CFG:
            if (param->ntf_en == PRF_CLI_START_NTF)
            {
                //DEVELOPER NOTE: SET boot_kb_in_report and boot_kb_out_report to REAL VALUE OF USER APPLICATION HERE
                app_hogpd_env->features[param->hids_nb].svc_features |= HOGPD_BOOT_KB_IN_NTF_CFG_MASK;
                ke_timer_set(APP_HOGPD_BOOT_KB_IN_REPORT_TIMER, TASK_APP, APP_HOGPD_BOOT_KB_IN_REPORT_TO);
            }
            else
            {
                app_hogpd_env->features[param->hids_nb].svc_features &= ~HOGPD_BOOT_KB_IN_NTF_CFG_MASK;
                ke_timer_clear(APP_HOGPD_BOOT_KB_IN_REPORT_TIMER, TASK_APP);
            }
            break;
        case HOGPD_BOOT_MOUSE_IN_REPORT_CFG:
            if (param->ntf_en == PRF_CLI_START_NTF)
            {
                //DEVELOPER NOTE: SET boot_mouse_in_report to REAL VALUE OF USER APPLICATION HERE
                app_hogpd_env->features[param->hids_nb].svc_features |= HOGPD_BOOT_MOUSE_IN_NTF_CFG_MASK;
                ke_timer_set(APP_HOGPD_BOOT_MOUSE_IN_REPORT_TIMER, TASK_APP, APP_HOGPD_BOOT_MOUSE_IN_REPORT_TO);
            }
            else
            {
                app_hogpd_env->features[param->hids_nb].svc_features &= ~HOGPD_BOOT_MOUSE_IN_NTF_CFG_MASK;
                ke_timer_clear(APP_HOGPD_BOOT_MOUSE_IN_REPORT_TIMER, TASK_APP);
            }
            break;
        case HOGPD_REPORT_CFG:
            if (param->ntf_en == PRF_CLI_START_NTF)
            {
                // only input report support notify property
                if (app_hogpd_env->features[param->hids_nb].report_char_cfg[param->report_nb] & HOGPD_CFG_REPORT_IN)
                {
                    //DEVELOPER NOTE: SET report to REAL VALUE OF USER APPLICATION HERE
                    app_hogpd_env->features[param->hids_nb].report_char_cfg[param->report_nb] |= HOGPD_REPORT_NTF_CFG_MASK;
                    ke_timer_set(APP_HOGPD_REPORT_TIMER, TASK_APP, APP_HOGPD_BOOT_KB_IN_REPORT_TO);
                }
            }
            else
            {
                app_hogpd_env->features[param->hids_nb].report_char_cfg[param->report_nb] &= ~HOGPD_REPORT_NTF_CFG_MASK;
                ke_timer_clear(APP_HOGPD_REPORT_TIMER, TASK_APP);
            }
            break;
        default:
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the protocol mode indication message      *//**
 *
 * @param[in] msgid     HOGPD_PROTO_MODE_IND
 * @param[in] param     Pointer to the struct hogpd_proto_mode_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called if a Protocol Mode characteristic value has been written
 * by a peer device.
 ****************************************************************************************
 */
int app_hogpd_proto_mode_ind_handler(ke_msg_id_t const msgid,
                                     struct hogpd_proto_mode_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    if (app_hogpd_env->conhdl == param->conhdl)
    {
        if (param->proto_mode == HOGP_BOOT_PROTOCOL_MODE)
        {
            app_hogpd_env->proto_mode[param->hids_nb] = HOGP_BOOT_PROTOCOL_MODE;
        }
        else
        {
            app_hogpd_env->proto_mode[param->hids_nb] = HOGP_REPORT_PROTOCOL_MODE;
        }
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the report indication message      *//**
 *
 * @param[in] msgid     HOGPD_REPORT_IND
 * @param[in] param     Pointer to the struct hogpd_report_info
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after the peer Host has written the value of one of the 
 * Report Characteristics.
 ****************************************************************************************
 */
int app_hogpd_report_ind_handler(ke_msg_id_t const msgid,
                                 struct hogpd_report_info *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the boot report indication message      *//**
 *
 * @param[in] msgid     HOGPD_BOOT_REPORT_IND
 * @param[in] param     Pointer to the struct hogpd_boot_report_info
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called aftera Boot Report has been written in the database.
 *
 ****************************************************************************************
 */
int app_hogpd_boot_report_ind_handler(ke_msg_id_t const msgid,
                                      struct hogpd_boot_report_info *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the control point indication message      *//**
 *
 * @param[in] msgid     HOGPD_CTNL_PT_IND
 * @param[in] param     Pointer to the struct hogpd_ctnl_pt_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called each time the host enables or disables sending of notifications 
 * for characteristic.
 ****************************************************************************************
 */
int app_hogpd_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                  struct hogpd_ctnl_pt_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("HOGPD receive instance(%d) control point %d.\r\n", param->hids_nb, param->hid_ctnl_pt);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the notify confirm message      *//**
 *
 * @param[in] msgid     HOGPD_NTF_SENT_CFM
 * @param[in] param     Pointer to the struct hogpd_ntf_sent_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPD
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after reception of the HOGPD_REPORT_UPD_REQ
 * or the HOGPD_BOOT_KB_IN_UPD_REQ or the HOGPD_BOOT_MOUSE_IN_UPD_REQ message to inform it
 * if a notificatio has been sent to the Host or if an error has been raised.
 * The following status code may be handled by application
 * - PRF_ERR_OK
 * - PRF_ERR_INVALID_PARAM
 * - PRF_ERR_REQ_DISALLOWED
 * - PRF_ERR_NTF_DISABLED
 * - PRF_ERR_FEATURE_NOT_SUPPORTED
 ****************************************************************************************
 */
int app_hogpd_ntf_sent_cfm_handler(ke_msg_id_t const msgid,
                                   struct hogpd_ntf_sent_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    switch (param->char_code)
    {
        case HOGPD_BOOT_KB_IN_REPORT_CHAR:
            app_hogpd_env->ntf_sending &= ~HOGPD_CFG_BOOT_KB_WR;
            break;
        case HOGPD_BOOT_MOUSE_IN_REPORT_CHAR:
            app_hogpd_env->ntf_sending &= ~HOGPD_CFG_BOOT_MOUSE_WR;
            break;
        case HOGPD_REPORT_CHAR:
            app_hogpd_env->ntf_sending &= ~HOGPD_CFG_REPORT_IN;
            break;
        default:
            break;
    }
    return (KE_MSG_CONSUMED);
}

/// @cond
/*
 ****************************************************************************************
 * @brief   
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_hogpd_boot_kb_in_report_timer_handler(ke_msg_id_t const msgid,
                                              void *param,
                                              ke_task_id_t const dest_id,
                                              ke_task_id_t const src_id)
{
    if (!(app_hogpd_env->ntf_sending & HOGPD_CFG_BOOT_KB_WR)) {
        app_hogpd_env->ntf_sending |= HOGPD_CFG_BOOT_KB_WR;
        for (uint8_t i = 0; i < app_hogpd_env->hids_nb; i++)
        {
            if (app_hogpd_env->features[i].svc_features & HOGPD_BOOT_KB_IN_NTF_CFG_MASK)
            {
                app_hogpd_boot_report_upd_req(app_hogpd_env->conhdl, i, HOGPD_BOOT_KB_IN_REPORT_CHAR, 
                                              sizeof(boot_kb_in_report), boot_kb_in_report);
            }
        }
    }
    ke_timer_set(APP_HOGPD_BOOT_KB_IN_REPORT_TIMER, TASK_APP, APP_HOGPD_BOOT_KB_IN_REPORT_TO);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief *//**
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_HOGPD).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_hogpd_boot_mouse_in_report_timer_handler(ke_msg_id_t const msgid,
                                                 void *param,
                                                 ke_task_id_t const dest_id,
                                                 ke_task_id_t const src_id)
{
    if (!(app_hogpd_env->ntf_sending & HOGPD_CFG_BOOT_MOUSE_WR)) {
        app_hogpd_env->ntf_sending |= HOGPD_CFG_BOOT_MOUSE_WR;
        for (uint8_t i = 0; i < app_hogpd_env->hids_nb; i++)
        {
            if (app_hogpd_env->features[i].svc_features & HOGPD_BOOT_MOUSE_IN_NTF_CFG_MASK)
            {
                app_hogpd_boot_report_upd_req(app_hogpd_env->conhdl, i, HOGPD_BOOT_MOUSE_IN_REPORT_CHAR, 
                                              sizeof(boot_mouse_in_report), boot_mouse_in_report);
            }
        }
    }
    ke_timer_set(APP_HOGPD_BOOT_MOUSE_IN_REPORT_TIMER, TASK_APP, APP_HOGPD_BOOT_MOUSE_IN_REPORT_TO);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief *//**
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_HOGPD).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_hogpd_report_timer_handler(ke_msg_id_t const msgid,
                                   void *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (!(app_hogpd_env->ntf_sending & HOGPD_CFG_REPORT_IN)) {
        app_hogpd_env->ntf_sending |= HOGPD_CFG_REPORT_IN;
        for (uint8_t i = 0; i < app_hogpd_env->hids_nb; i++)
        {
            for (uint8_t j = 0; j < app_hogpd_env->features[i].report_nb; j++) {
                if (app_hogpd_env->features[i].report_char_cfg[j] & HOGPD_REPORT_NTF_CFG_MASK) {
                    app_hogpd_report_upd_req(app_hogpd_env->conhdl, i, j, sizeof(in_report), in_report);
                }
            }
        }
    }
    ke_timer_set(APP_HOGPD_REPORT_TIMER, TASK_APP, APP_HOGPD_REPORT_TO);
    return (KE_MSG_CONSUMED);
}
/// @endcond

#endif // BLE_HID_DEVICE
