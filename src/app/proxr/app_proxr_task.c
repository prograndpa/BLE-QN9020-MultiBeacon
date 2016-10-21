/**
 ****************************************************************************************
 *
 * @file app_proxr_task.c
 *
 * @brief Application PROXR implementation
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
 * @addtogroup APP_PROXR_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_PROX_REPORTER
#include "app_proxr.h"
#include "usr_design.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_proxr_env_tag *app_proxr_env = &app_env.proxr_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the PROXR.      *//**
 *
 * @param[in] msgid     PROXR_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct proxr_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXR
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after reception of create database. The status parameter 
 * indicates if the services have been successfully added (ATT_ERR_NO_ERROR) or not 
 * (ATT_INSUFF_RESOURCE).
 ****************************************************************************************
 */
int app_proxr_create_db_cfm_handler(ke_msg_id_t const msgid,
                                struct proxr_create_db_cfm *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_PROX_REPORTER_BIT);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the PROXR.     *//**
 *
 * @param[in] msgid     PROXR_DISABLE_IND
 * @param[in] param     Pointer to the struct proxr_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXR
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of the correct disable or the Reporter role,
 * and to give the application the LLS alert level to save until the next activation of the 
 * Reporter role.
 ****************************************************************************************
 */
int app_proxr_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct proxr_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_proxr_env->conhdl = 0xFFFF;
    app_proxr_env->link_loss_lvl = param->lls_alert_lvl;
    app_proxr_env->enabled = false;
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles alert indication from the PROXR.  *//**
 *
 * @param[in] msgid     PROXR_ALERT_IND
 * @param[in] param     Pointer to the struct proxr_alert_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXR
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to request the Application to start the alert on the device 
 * considering the indicated alert level. The handler may be triggered on two conditions:
 * The IAS alert level characteristic has been written to a valid value, in which case
 * alert_lvl will be set to the IAS alert level value.
 * A disconnection with a reason other than the normal local/remote link terminations has
 * been received, in which case alert_lvl will be set to the LLS alert level value.
 * The Application actions following reception of this indication are strictly implementation
 * specific (it may try to reconnect to the peer and stop alert upon that, or timeout the
 * alert after acertain time, please see the specification)
 ****************************************************************************************
 */ 
int app_proxr_alert_ind_handler(ke_msg_id_t const msgid,
                                struct proxr_alert_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);

    if (param->char_code == PROXR_LLS_CHAR)
    {
        app_proxr_env->link_loss_lvl = param->alert_lvl;
        QPRINTF("PROXR LLS lvl %d.\r\n", param->alert_lvl);
    }
    else if (param->char_code == PROXR_IAS_CHAR)
    {
        app_proxr_env->alert_lvl = param->alert_lvl;
        QPRINTF("PROXR IAS lvl %d.\r\n", param->alert_lvl);
    }   

    return (KE_MSG_CONSUMED);
}

#endif // BLE_PROX_REPORTER

/// @} APP_PROXR_TASK
