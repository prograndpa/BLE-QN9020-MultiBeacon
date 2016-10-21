/**
 ****************************************************************************************
 *
 * @file app_htpt_task.c
 *
 * @brief Application HTPT implementation
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
 * @addtogroup APP_HTPT_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#if BLE_HT_THERMOM
#include "app_htpt.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_htpt_env_tag *app_htpt_env = &app_env.htpt_ev;
/// @endcond

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the HTPT.       *//**
 *
 * @param[in] msgid     HTPT_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct htpt_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after reception of a database creation.
 * The status parameter indicates if the HTS has been successfully added (ATT_ERR_NO_ERROR)
 * or not (ATT_INSUFF_RESOURCE).
 ****************************************************************************************
 */
int app_htpt_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct htpt_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_HT_THERMOM_BIT);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the HTPT.      *//**
 *
 * @param[in] msgid     HTPT_DISABLE_IND
 * @param[in] param     Pointer to the struct htpt_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The configuration 
 * that the collector has set in HTS attributes must be conserved and the 4 values that are
 * important are sent back to the application for safe keeping until the next time this 
 * profile role is enabled.
 ****************************************************************************************
 */
int app_htpt_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct htpt_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_htpt_env->conhdl = 0xffff;
    app_htpt_env->enabled = false;
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication from the HTPT.       *//**
 *
 * @param[in] msgid     HTPT_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is to inform the Application of an occurred error.
 ****************************************************************************************
 */
int app_htpt_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("HTPT error indication.\r\n");
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the send temperature confirm from the HTPT.       *//**
 *
 * @param[in] msgid     HTPT_TEMP_SEND_CFM
 * @param[in] param     Pointer to the struct htpt_temp_send_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called when the status of a notification request sent by application 
 * for the Intermediate Temperature Char return. 
 * The importance of a confirmation of a specific indication is that the application can erase
 * the stable temperature values that it sent the peer successfully. For intermediate temperatures
 * the confirmation is useless, so they would not be stored.
 ****************************************************************************************
 */
int app_htpt_temp_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct htpt_temp_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the send measurement interval indication from the HTPT.       *//**
 *
 * @param[in] msgid     HTPT_MEAS_INTV_CHG_IND
 * @param[in] param     Pointer to the struct htpt_meas_intv_chg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the application that the measurement interval value has changed.
 * The application uses the new value to either decide to stop periodic measurements if the 
 * value of the interval has changed from non 0 to 0, or the opposite, to start periodic 
 * measurements using the interval value, if the value has changed from 0 to non 0.
 * This handler will only be triggered if the new value that the Collector is trying to write is valid
 * (within the Valid Range descriptor minimum and maximum values). If the value is not within
 * range, this handler is never be triggered by the application because the HTPT will send an Error
 * Response to the Collector with the 'Out of Range' code 0x80 and the new value will never be
 * set.
 ****************************************************************************************
 */
int app_htpt_meas_intv_chg_ind_handler(ke_msg_id_t const msgid,
                                       struct htpt_meas_intv_chg_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // We need check the valid range of interval
    if (param->intv >= HTPT_MEAS_INTV_DFLT_MIN &&
        param->intv <= HTPT_MEAS_INTV_DFLT_MAX)
    {
        app_htpt_env->meas_intv = param->intv;
        QPRINTF("HTPT interval change indication.\r\n");
        app_task_msg_hdl(msgid, param);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the configure ind/ntf indication from the HTPT.       *//**
 *
 * @param[in] msgid     HTPT_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct htpt_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPT
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a new value set in one of the 3
 * Client Characteristic Configuration Descriptors in HTS. It allows the application to be aware of
 * its current settings for HTS and to alter its behavior accordingly if the implementation desires
 * it.
 ****************************************************************************************
 */
int app_htpt_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct htpt_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);
    return (KE_MSG_CONSUMED);
}

#endif // BLE_HT_THERMOM

/// @} APP_HTPT_TASK
