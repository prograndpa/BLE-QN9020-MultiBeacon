/**
 ****************************************************************************************
 *
 * @file app_findt_task.c
 *
 * @brief Application FINDT task implementation
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
#if BLE_FINDME_TARGET
#include "app_findt.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_findt_env_tag *app_findt_env = &app_env.findt_ev;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the create DB confirmation from the FINDT.    *//**
 *
 * @param[in] msgid     FINDT_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct findt_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation.
 * The status parameter indicates if the IAS has been successfully added or not.
 * Possible values for the status are: ATT_ERR_NO_ERROR and ATT_INSUFF_RESOURCE.
 ****************************************************************************************
 */
int app_findt_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct findt_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_FINDME_TARGET_BIT);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the alert indication from the FINDT.    *//**
 *
 * @param[in] msgid     FINDT_ALERT_IND
 * @param[in] param     Pointer to the struct findt_alert_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is calld to inform the Application of a valid alert level written by the 
 * peer in the IAS Alert Level Characteristic.  
 * Possible values are: No Alert(0), Mild Alert(1), High Alert(2)
 * The Application alone is responsible for actually triggering/stopping a noticeable visual/audio
 * alert on the device upon reception of message FINDT_ALERT_IND.
 ****************************************************************************************
 */
int app_findt_alert_ind_handler(ke_msg_id_t const msgid,
                                struct findt_alert_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if (param->conhdl == app_findt_env->conhdl)
    {
        app_findt_env->alert_lvl = param->alert_lvl;
        QPRINTF("FINDT alert level %d.\r\n", param->alert_lvl);
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the FINDT.    *//**
 *
 * @param[in] msgid     FINDT_DISABLE_IND
 * @param[in] param     Pointer to the struct findt_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is calld to inform the application of a correct disable. It will be
 * triggered after a disconnection with the peer device.
 ****************************************************************************************
 */
int app_findt_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct findt_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);
    app_findt_env->enabled = false;
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the FINDT.    *//**
 *
 * @param[in] msgid     FINDT_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application of an occurred error information
 ****************************************************************************************
 */
int app_findt_error_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_server_error_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

#endif // BLE_FINDME_TARGET

