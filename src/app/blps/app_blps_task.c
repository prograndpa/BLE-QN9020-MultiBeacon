/**
 ****************************************************************************************
 *
 * @file app_blps_task.c
 *
 * @brief Application BLPS task implementation
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
 * @addtogroup APP_BLPS_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_BP_SENSOR)  
#include "app_blps.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_blps_env_tag *app_blps_env = &app_env.blps_ev;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Handles create database confirm message from the BLPS task     *//**
 *
 * @param[in] msgid     BLPS_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct blps_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation. It contains status of database creation.
 ****************************************************************************************
 */
int app_blps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct blps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if(param->status == ATT_ERR_NO_ERROR) 
    {
        app_clear_local_service_flag(BLE_BP_SENSOR_BIT);
    }
    return (KE_MSG_CONSUMED);
}
 
/*
 ****************************************************************************************
 * @brief Handles disable service indication from the BLPS task     *//**
 *
 * @param[in] msgid     BLPS_DISABLE_IND
 * @param[in] param     Pointer to the struct blps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The configuration 
 * that the collector has set in BPS attributes must be conserved and the 4 values that 
 * are important are sent back to the application for safe keeping until the next time 
 * this profile role is enabled.
 ****************************************************************************************
 */
int app_blps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct blps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_blps_env->enabled = false;
    app_blps_env->ntf_sending = 0;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the configure ind/ntf indication from the BLPS task     *//**
 *
 * @param[in] msgid     BLPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct blps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 ****************************************************************************************
 */
int app_blps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct blps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF(("BLPS receive configure indication.\r\n"));

    if(param->char_code == BPS_BP_MEAS_CODE) 
    {
        if (param->cfg_val == PRF_CLI_START_IND)
        {
            app_blps_env->evt_cfg |= BLPS_BP_MEAS_IND_CFG;
        }
        else
        {
            app_blps_env->evt_cfg &= ~BLPS_BP_MEAS_IND_CFG;
        }
    }
    else if (param->char_code == BPS_INTERM_CP_CODE)
    {
        if (param->cfg_val == PRF_CLI_START_NTF)
        {
            app_blps_env->evt_cfg |= BLPS_INTM_CUFF_PRESS_NTF_CFG;
        }
        else
        {
            app_blps_env->evt_cfg &= ~BLPS_INTM_CUFF_PRESS_NTF_CFG;
        }
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the blood pressure measurement value confirm message from the BLPS *//**
 *
 * @param[in] msgid     BLPS_MEAS_SEND_CFM 
 * @param[in] param     Pointer to the struct blps_meas_send_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform to the application a confirmation, or error status of 
 * a notification request being sent to GATT for the Intermediate Cuff Pressure Char.
 ****************************************************************************************
 */
int app_blps_meas_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct blps_meas_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    app_blps_env->ntf_sending = false;
    app_task_msg_hdl(msgid, param);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication message from the BLPS task   *//**
 *
 * @param[in] msgid     BLPS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of an occurred error information
 ****************************************************************************************
 */
int app_blps_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF(("BLPS error indication.\r\n"));
    return (KE_MSG_CONSUMED);
}

#endif // BLE_BP_SENSOR

/// @} APP_BLPS_TASK
