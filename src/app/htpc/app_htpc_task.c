/**
 ****************************************************************************************
 *
 * @file app_htpc_task.c
 *
 * @brief Application HTPC task implementation
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
 * @addtogroup APP_HTPC_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HT_COLLECTOR
#include "app_htpc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_htpc_env_tag *app_htpc_env = &app_env.htpc_ev[0];
/// @endcond


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HTPC. *//**
 *
 * @param[in] msgid     HTPC_ENABLE_CFM
 * @param[in] param     Pointer to struct htpc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector to either send the discovery results of 
 *  HTS on the Thermometer and confirm enabling of the Collector role, or to simply confirm 
 *  enabling of Collector role if it is a normal connection and the attribute details are already 
 *  known. 
 *
 ****************************************************************************************
 */
int app_htpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct htpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HTPC enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_htpc_env[idx].conhdl = param->conhdl;
        app_htpc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the HTPC. *//**
 *
 * @param[in] msgid     HTPC_ERROR_IND
 * @param[in] param     Pointer to struct htpc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of an 
 *  error occurred in different situations. The error codes are specific to this profile and    
 *  defined in htpc.h. An error may occur during attribute discovery or due to application request 
 *  parameters. Following reception of this message, the application will decide the necessary 
 *  action.
 *
 ****************************************************************************************
 */
int app_htpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct htpc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HTPC error indication status: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     HTPC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct htpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a 
 *  received read response. The status and the data from the read response are passed directly to  
 *  Application, which must interpret them based on the request it made.
 *
 ****************************************************************************************
 */
int app_htpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct htpc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HTPC read char response status: 0x%x code: 0x%X.\r\n",
        param->status,
        param->att_code);

    if (param->att_code == HTPC_CHAR_HTS_MEAS_INTV)
    {
        QPRINTF("MEAS INTV: ");
        if (param->data.len == 0)
        {
            QPRINTF("NULL");
        }
        else
        {
            for (uint8_t i = 0; i < param->data.len; i++)
                QPRINTF("%x", param->data.data[param->data.len-i-1]);
        }
        QPRINTF("s\r\n");
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     HTPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct htpc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a 
 *  received write response. The status and the data from the write response are passed directly 
 *  to Application, which must interpret them based on the request it made.
 *
 ****************************************************************************************
 */
int app_htpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct htpc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HTPC write char response status: 0x%x.\r\n", param->status);

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the temperature value send to APP. *//**
 *
 * @param[in] msgid     HTPC_TEMP_IND
 * @param[in] param     Pointer to struct htpc_temp_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a 
 *  received temperature value, either by notification (flag_stable_meas = intermediate) or 
 *  indication (flag_stable_meas = stable). No confirmation of reception is needed because the 
 *  GATT sends it directly to the peer. 
 * @note
 *
 *  get temperature value, 32bits float by conversion rule: 
 *  - first byte: exponent (signed)
 *  - following three bytes: integer
 *  - for example: 0xff000173 = 371*10(expo: -1) = 37.1 Celsius
 *
 ****************************************************************************************
 */
int app_htpc_temp_ind_handler(ke_msg_id_t const msgid,
                      struct htpc_temp_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
#if QN_DBG_TRACE_MORE 
    QPRINTF("Date: %04d:%02d:%02d Time: %02d:%02d:%02d\r\n", 
        param->temp_meas.time_stamp.year,
        param->temp_meas.time_stamp.month,
        param->temp_meas.time_stamp.day,
        param->temp_meas.time_stamp.hour,
        param->temp_meas.time_stamp.min,
        param->temp_meas.time_stamp.sec);
#endif

    QPRINTF("Temperature Value(%d): 0x%04x\r\n", param->flag_stable_meas, param->temp_meas.temp);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the measurement interval value indication to APP. *//**
 *
 * @param[in] msgid     HTPC_MEAS_INTV_IND
 * @param[in] param     Pointer to struct htpc_meas_intv_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a 
 *  received Measurement Interval Char. Indication and the value it indicates. This value should be 
 *  used by the Application as seen fit. No response is necessary (the GATT sends the necessary 
 *  confirmation to the Indication PDU).
 *
 ****************************************************************************************
 */
int app_htpc_meas_intv_ind_handler(ke_msg_id_t const msgid,
                      struct htpc_meas_intv_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HTPC measurement interval value indication interval: 0x%02X.\r\n", 
        param->intv);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     HTPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Health Thermometer Profile Client 
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_htpc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("HTPC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}

#endif
/// @} APP_HTPC_TASK
