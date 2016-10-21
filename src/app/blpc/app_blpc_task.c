/**
 ****************************************************************************************
 *
 * @file app_blpc_task.c
 *
 * @brief Application BLPC task implementation
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
 * @addtogroup APP_BLPC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_BP_COLLECTOR
#include "app_blpc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_blpc_env_tag *app_blpc_env = &app_env.blpc_ev[0];
/// @endcond


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the BLPC. *//**
 *
 * @param[in] msgid     BLPC_ENABLE_CFM
 * @param[in] param     Pointer to struct blpc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector to either send the discovery results of BPS on the Blood 
 *  Pressure and confirm enabling of the Collector role, or to simply confirm enabling of Collector role if it is a normal 
 *  connection and the attribute details are already known.
 ****************************************************************************************
 */
int app_blpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct blpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BLPC enable confirmation status: 0x%X.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);

        app_blpc_env[idx].conhdl = param->conhdl;
        app_blpc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the BLPC.  *//**
 *
 * @param[in] msgid     BLPC_ERROR_IND
 * @param[in] param     Pointer to struct blpc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of an error occurred in different 
 *  situations. The error codes are proprietary and defined in prf_types.h. An error may occur during attribute 
 *  discovery or due to application request parameters. Following reception of this message, the application will decide 
 *  the necessary action.
 *
 ****************************************************************************************
 */
int app_blpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct blpc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BLPC error indication status: 0x%X.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     BLPC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct blpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received read response. The 
 *  status and the data from the read response are passed directly to Application, which must interpret them based on 
 *  the request it made.
 ****************************************************************************************
 */
int app_blpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct blpc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BLPC read char response status: 0x%x.\r\n", param->status);
    QTRACE(param->data.data, param->data.len, 1, 2);
    QPRINTF("\r\n");
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     BLPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct blpc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received write response. The 
 *  status and the data from the write response are passed directly to Application, which must interpret them based on 
 *  the request it made. 
 ****************************************************************************************
 */
int app_blpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct blpc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BLPC write char response status: 0x%X.\r\n", param->status);

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Blood pressure value send to APP. *//**
 *
 * @param[in] msgid     BLPC_BP_MEAS_IND
 * @param[in] param     Pointer to struct blpc_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received blood pressure 
 *  value, either by notification (flag_interm_cp = intermediate) or indication (flag_interm_cp = stable). The application 
 *  will do what it needs to do with the received measurement. No confirmation of reception is needed because the GATT 
 *  sends it directly to the peer. 
 ****************************************************************************************
 */
int app_blpc_bp_meas_ind_handler(ke_msg_id_t const msgid,
                      struct blpc_meas_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
#if QN_DBG_TRACE_MORE  
    if (param->meas_val.flags & BPS_FLAG_TIME_STAMP_PRESENT)
        QPRINTF("Date: %04d:%02d:%02d Time: %02d:%02d:%02d\r\n", 
            param->meas_val.time_stamp.year,
            param->meas_val.time_stamp.month,
            param->meas_val.time_stamp.day,
            param->meas_val.time_stamp.hour,
            param->meas_val.time_stamp.min,
            param->meas_val.time_stamp.sec);
#endif

    QPRINTF("BP Value(%d), Flag:%X, User ID:%X, Meas Status:%X, Pulse Rate:%d\r\n",
        param->flag_interm_cp,
        param->meas_val.flags,
        (param->meas_val.flags & BPS_FLAG_USER_ID_PRESENT) ? param->meas_val.user_id : 0,
        (param->meas_val.flags & BPS_FLAG_MEAS_STATUS_PRESENT) ? param->meas_val.meas_status : 0,
        (param->meas_val.flags & BPS_FLAG_PULSE_RATE_PRESENT) ? param->meas_val.pulse_rate : 0
        );

#if QN_DBG_TRACE_MORE    
    if (param->meas_val.flags & BPS_FLAG_KPA)
        QPRINTF("Systolic:0x%X(kpa), Diastolic:0x%X(kpa), Mean Arterial Pressure:0x%X(kpa)\r\r\n",
            param->meas_val.systolic,
            param->meas_val.diastolic,
            param->meas_val.mean_arterial_pressure
            );
    else
        QPRINTF("Systolic:%d(mmhg), Diastolic:%d(mmhg), Mean Arterial Pressure:%d(mmhg)\r\r\n",
            param->meas_val.systolic,
            param->meas_val.diastolic,
            param->meas_val.mean_arterial_pressure
            );
#endif
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     BLPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Blood Pressure Service Client
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_blpc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("BLPC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_BLPC_TASK
