/**
 ****************************************************************************************
 *
 * @file app_scpps_task.c
 *
 * @brief Application SCPPS implementation
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
 * @addtogroup APP_SCPPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"
#if BLE_SP_SERVER
#include "app_scpps.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_scpps_env_tag *app_scpps_env = &app_env.scpps_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the SCPPS.      *//**
 *
 * @param[in] msgid     SCPPS_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct scpps_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation. It contains status of database creation.
 ****************************************************************************************
 */
int app_scpps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct scpps_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_SP_SERVER_BIT);
    }
    return (KE_MSG_CONSUMED);
}   

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the SCPPS.     *//**
 *
 * @param[in] msgid     SCPPS_DISABLE_IND
 * @param[in] param     Pointer to the struct scpps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application of a corrent disable. The current 
 * notification confiuration value for the Scan Refresh characteristic are included in
 * the paramerers so that the higher application may safely keep the configuration until
 * the next time the profile is enabled.
 ****************************************************************************************
 */
int app_scpps_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct scpps_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_scpps_env->conhdl = 0xFFFF;
    app_scpps_env->enabled = false;
    app_scpps_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the eror indication message from the SCPPS.       *//**
 *
 * @param[in] msgid     SCPPS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of occurred error information
 ****************************************************************************************
 */
int app_scpps_error_ind_handler(ke_msg_id_t const msgid,
                                struct prf_server_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("SCPPS error indication.\r\n");
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the scan interval indication message from the SCPPS.      *//**
 *
 * @param[in] msgid     SCPPS_SCAN_INTV_WD_IND
 * @param[in] param     Pointer to the struct scpps_scan_intv_wd_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered when the Scan Interval Window Characteristic value
 * has been written by the peer device.
 ****************************************************************************************
 */
int app_scpps_scan_intv_wd_ind_handler(ke_msg_id_t const msgid,
                                       struct scpps_scan_intv_wd_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    app_scpps_env->scan_intv_wd = param->scan_intv_wd;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the scan refresh notification configuration message from the SCPPS.   *//**
 *
 * @param[in] msgid     SCPPS_SCAN_REFRESH_NTF_CFG_IND
 * @param[in] param     Pointer to the struct scpps_scan_refresh_ntf_cfg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered when the peer device has enabled or disabled sending of
 * notiications for the Scan Refresh Characteristic.
 ****************************************************************************************
 */
int app_scpps_scan_refresh_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                               struct scpps_scan_refresh_ntf_cfg_ind *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
    if (param->scan_refresh_ntf_en == PRF_CLI_START_NTF)
    {
        app_scpps_env->scan_refresh_ntf_cfg = PRF_CLI_START_NTF;        
    }
    else
    {
        app_scpps_env->scan_refresh_ntf_cfg = PRF_CLI_STOP_NTFIND;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the scan refresh confirm message from the SCPPS.      *//**
 *
 * @param[in] msgid     SCPPS_SCAN_REFRESH_SEND_CFM
 * @param[in] param     Pointer to the struct scpps_scan_refresh_send_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered if the Scan Refresh Characteristic value has been 
 * notified or not.
 ****************************************************************************************
 */
int app_scpps_scan_refresh_send_cfm_handler(ke_msg_id_t const msgid,
                                            struct scpps_scan_refresh_send_cfm *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    if (param->status != PRF_ERR_OK)
    {
        app_scpps_env->scan_refresh_ntf_cfg = PRF_CLI_STOP_NTFIND;
        QPRINTF("SCPPS scan refresh confirmation error %d.\r\n", param->status);
    }
    app_scpps_env->ntf_sending = false;
    return (KE_MSG_CONSUMED);
}

#endif // BLE_SP_SERVER

/// @} APP_SCPPS_TASK
