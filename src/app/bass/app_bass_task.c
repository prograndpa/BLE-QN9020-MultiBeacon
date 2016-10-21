/**
 ****************************************************************************************
 *
 * @file app_bass_task.c
 *
 * @brief Application BASS implementation
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
 * @addtogroup APP_BASS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_BATT_SERVER
#include "app_bass.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_bass_env_tag *app_bass_env = &app_env.bass_ev;

/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the BASS.   *//**
 *
 * @param[in] msgid     BASS_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct bass_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation. The status of parameter may have
 * the following values:
 * - PRF_ERR_OK
 * - PRF_ERR_INVALID_PARAM
 * - ATT_INSUFF_RESOURCE
 ****************************************************************************************
 */
int app_bass_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct bass_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_BATT_SERVER_BIT);
    }
    #if BLE_HID_DEVICE
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
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the BASS.  *//**
 *
 * @param[in] msgid     BASS_DISABLE_IND
 * @param[in] param     Pointer to the struct bass_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. 
 ****************************************************************************************
 */
int app_bass_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct bass_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_bass_env->conhdl = 0xFFFF;
    app_bass_env->enabled = false;
    app_bass_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication from the BASS.   *//**
 *
 * @param[in] msgid     BASS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered if an error has been raised during the communication.
 ****************************************************************************************
 */
int app_bass_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("BASS error indication.\r\n");
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the battery level update confirm message from the BASS.       *//**
 *
 * @param[in] msgid     BASS_BATT_LEVEL_UPD_CFM
 * @param[in] param     Pointer to the struct bass_batt_level_upd_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered if a notification has been sent to the peer device.
 ****************************************************************************************
 */
int app_bass_batt_level_upd_cfm_handler(ke_msg_id_t const msgid,
                                        struct bass_batt_level_upd_cfm *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    app_bass_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the battery level update confirm message from the BASS.       *//**
 *
 * @param[in] msgid     BASS_BATT_LEVEL_NTF_CFG_IND
 * @param[in] param     Pointer to the struct bass_batt_level_ntf_cfg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered when the notification configuration has been modified
 * for one of the Battery Level Characteristics.
 ****************************************************************************************
 */
int app_bass_batt_level_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                            struct bass_batt_level_ntf_cfg_ind *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    if (param->ntf_cfg == PRF_CLI_START_NTF)
    {
        // Ntf cfg bit set to 1
        app_bass_env->features[param->bas_instance] |= BASS_FLAG_NTF_CFG_BIT;
    }
    else
    {
        // Ntf cfg bit set to 0
        app_bass_env->features[param->bas_instance] &= ~BASS_FLAG_NTF_CFG_BIT;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

#endif // BLE_BATT_SERVER

/// @} APP_BASS_TASK
