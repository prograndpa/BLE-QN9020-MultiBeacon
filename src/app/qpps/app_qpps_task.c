/**
 ****************************************************************************************
 *
 * @file app_qpps_task.c
 *
 * @brief Application QPPS implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_QPPS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_QPP_SERVER
#include "app_qpps.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_qpps_env_tag *app_qpps_env = &app_env.qpps_ev;

static void app_test_send_data(uint8_t);

/*
 ****************************************************************************************
 * @brief Calculate the number of non-zero bit.
 *
 ****************************************************************************************
 */
static uint32_t get_bit_num(uint32_t val)
{
    uint32_t bit_cnt = 0;

    while (val != 0)
    {
        if (val & 0x1)
            bit_cnt++;
        val >>= 1;
    }
    return bit_cnt;
}

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the QPPS.       *//**
 *
 * @param[in] msgid     QPPS_CREATE_DB_CFM
 * @param[in] param     struct qpps_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered after a database creation. It contains status of database creation.
 *
 ****************************************************************************************
 */
int app_qpps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct qpps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR) 
    {
        app_clear_local_service_flag(BLE_QPPS_SERVER_BIT);
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable database indication from the QPPS.      *//**
 *
 * @param[in] msgid     QPPS_DISABLE_IND
 * @param[in] param     Pointer to the struct qpps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of a correct disable. The configuration
 * that the client has set in ntf_en field must be conserved for bonded devices.
 *
 ****************************************************************************************
 */
int app_qpps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct qpps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_qpps_env->conhdl = 0xFFFF;
    app_qpps_env->enabled = false;
    app_qpps_env->features = 0;
    app_qpps_env->char_status = 0;
    #if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)
    app_qpps_env->tx_buffer_available = QPPS_TX_BUFFER_SIZE;
    #endif

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication nessage from the QPPS.        *//**
 *
 * @param[in] msgid     QPPS_ERROR_IND
 * @param[in] param     Pointer to the struct qpps_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of an occurred error.
 * 
 ****************************************************************************************
 */
int app_qpps_error_ind_handler(ke_msg_id_t const msgid,
                               struct qpps_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("QPPS Error indication.\r\n");
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the send data confirm message from the QPPS.     *//**
 *
 * @param[in] msgid     QPPS_DATA_SEND_CFM
 * @param[in] param     Pointer to the struct qpps_data_send_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report to the application a confirmation, or error status of a notification
 * request being sent by application.
 *
 ****************************************************************************************
 */
int app_qpps_data_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct qpps_data_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (app_qpps_env->conhdl == param->conhdl && param->status == PRF_ERR_OK)
    {
        #if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)
        app_qpps_env->tx_buffer_available++;
        #else
        app_qpps_env->char_status |= (1 << param->char_index);
        #endif
    }
#if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)
    else if (app_qpps_env->conhdl == param->conhdl && param->status == GATT_NOTIFY_GET_DATA)
    {
        app_qpps_env->char_status |= (1 << param->char_index);
    }
    else if(param->status ==PRF_ERR_NTF_DISABLED)
    {
        app_qpps_env->tx_buffer_available++;
        return (KE_MSG_CONSUMED);
    }
#endif
    else
    {
        QPRINTF("QPPS send error %d.\r\n", param->status);
        return (KE_MSG_CONSUMED);
    }
    // Send next group data until current data have been sent
    if (get_bit_num(app_qpps_env->char_status) >= 1)
    {
        #if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)   
        if(app_qpps_env->tx_buffer_available)
        #endif 
        app_test_send_data(get_bit_num(app_qpps_env->char_status));
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the QPPS.     *//**
 *
 * @param[in] msgid     QPPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct qpps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 *
 ****************************************************************************************
 */
int app_qpps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct qpps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (app_qpps_env->conhdl == param->conhdl)
    {
        if (param->cfg_val == PRF_CLI_START_NTF)
        {
            app_qpps_env->features |= (QPPS_VALUE_NTF_CFG << param->char_index);
            // App send data if all of characteristic have been configured
            if (get_bit_num(app_qpps_env->features) == app_qpps_env->tx_char_num)
            {
                app_qpps_env->char_status = app_qpps_env->features;
                app_test_send_data(app_qpps_env->tx_char_num);
            }
        }
        else
        {
            app_qpps_env->features &= ~(QPPS_VALUE_NTF_CFG << param->char_index);
            app_qpps_env->char_status &= ~(QPPS_VALUE_NTF_CFG << param->char_index);
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the data ind message from the QPPS.       *//**
 *
 * @param[in] msgid     QPPS_DAVA_VAL_IND
 * @param[in] param     Pointer to the struct qpps_data_val_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to handle the data sent form peer device
 *
 ****************************************************************************************
 */
int app_qpps_data_ind_handler(ke_msg_id_t const msgid,
                              struct qpps_data_val_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    if (param->length > 0)
    {
        QPRINTF("len=%d, I%02X", param->length, param->data[0]);
    }
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

/// @cond
/*
 ****************************************************************************************
 * @brief Send test data.
 *
 * @param[in] max The maximum times used to send data
 *
 ****************************************************************************************
 */
static void app_test_send_data(uint8_t max)
{
    uint8_t cnt;

    max = max;
    for (cnt = 0; cnt < app_qpps_env->tx_char_num; cnt++)
    {
        if ((app_qpps_env->char_status >> cnt) & QPPS_VALUE_NTF_CFG)
        {
            static uint8_t val[] = {0, '0', '1', '2','3','4','5','6','7','8','9','8','7','6','5','4','3','2','1','0'};

            // Increment the first byte for test 
            val[0]++;

            #if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)   
            app_qpps_env->tx_buffer_available--;
            #endif 
            // Allow next notify until confirmation received in this characteristic
            app_qpps_env->char_status &= ~(QPPS_VALUE_NTF_CFG << cnt);
            app_qpps_data_send(app_qpps_env->conhdl, cnt, sizeof(val), val);
        }
    }
}

/// @endcond

#endif // BLE_QPP_SERVER

/// @} APP_QPPS_TASK
