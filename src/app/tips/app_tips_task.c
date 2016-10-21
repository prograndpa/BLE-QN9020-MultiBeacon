/**
 ****************************************************************************************
 *
 * @file app_tips_task.c
 *
 * @brief Application TIPS implementation
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
 * @addtogroup APP_TIPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_TIP_SERVER
#include "app_tips.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_tips_env_tag *app_tips_env = &app_env.tips_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the TIPS.       *//**
 *
 * @param[in] msgid     TIPS_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct tips_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler shall be called after reception of a database creation. The status parameter 
 * indicates if requested services have been successfully added into the database or not.
 ****************************************************************************************
 */
int app_tips_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct tips_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_TIP_SERVER_BIT);
    }
    return (KE_MSG_CONSUMED);
}                               

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the TIPS.     *//**
 *
 * @param[in] msgid     TIPS_DISABLE_IND
 * @param[in] param     Pointer to the struct tips_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application of a correct disable.
 * The current notification configuration value for the Current Time characteristic is included in
 * the parameters so that the higher application may safely keep the configuration until the next
 * time the profile role is enabled.
 ****************************************************************************************
 */
int app_tips_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct tips_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_tips_env->conhdl = 0xFFFF;
    app_tips_env->enabled = false;
    app_tips_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the eror indication message from the TIPS.      *//**
 *
 * @param[in] msgid     TIPS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of an occurred error.
 ****************************************************************************************
 */
int app_tips_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("TIPS error code %d indication.\r\n", param->status);
    switch (param->msg_id)
    {
        case TIPS_UPD_CURR_TIME_REQ:
            // last notification complete
            app_tips_env->ntf_sending = false;
            break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the current time CCC indication message from the TIPS.    *//**
 *
 * @param[in] msgid     TIPS_CURRENT_TIME_CCC_IND
 * @param[in] param     Pointer to the struct tips_current_time_ccc_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application about a modification of the Current Time
 * Client Configuration characteristic value.
 ****************************************************************************************
 */
int app_tips_current_time_ccc_ind_handler(ke_msg_id_t const msgid,
                                          struct tips_current_time_ccc_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    if (param->cfg_val == PRF_CLI_START_NTF) 
    {
        app_tips_env->features |= TIPS_CTS_CURRENT_TIME_CFG;
    }
    else
    {
        app_tips_env->features &= ~TIPS_CTS_CURRENT_TIME_CFG;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the time update control indication message from the TIPS.     *//**
 *
 * @param[in] msgid     TIPS_TIME_UPD_CTNL_PT_IND
 * @param[in] param     Pointer to the struct tips_time_upd_ctnl_pt_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application about a modification of the Time Update
 * Control Point Characteristic value.
 ****************************************************************************************
 */
int app_tips_time_upd_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                          struct tips_time_upd_ctnl_pt_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    if (app_tips_env->features & TIPS_RTUS_SUP)
    {
        app_task_msg_hdl(msgid, param);
    }    

    return (KE_MSG_CONSUMED);
}

#endif // BLE_TIP_SERVER

/// @} APP_TIPS_TASK
