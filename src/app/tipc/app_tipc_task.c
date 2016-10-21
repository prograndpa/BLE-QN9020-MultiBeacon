/**
 ****************************************************************************************
 *
 * @file app_tipc_task.c
 *
 * @brief Application TIPC task implementation
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
 * @addtogroup APP_TIPC_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_TIP_CLIENT
#include "app_tipc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_tipc_env_tag *app_tipc_env = &app_env.tipc_ev[0];
/// @endcond


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the TIPC. *//**
 *
 * @param[in] msgid     TIPC_ENABLE_CFM
 * @param[in] param     Pointer to struct tipc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client to either send the discovery results of CTS, NDCS or 
 *  RTUS and confirm enabling of the Client role (status = PRF_ERR_OK), or to simply confirm 
 *  enabling of Client role if it is a normal connection and the attribute details are already known 
 *  (status = PRF_ERR_OK), or to inform the application that the discovery process has been 
 *  stopped because of a missing attribute (status = PRF_ERR_STOP_DISC_CHAR_MISSING).
 *
 ****************************************************************************************
 */
int app_tipc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct tipc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC enable confirmation status: 0x%X.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR)
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_tipc_env[idx].conhdl = param->conhdl;
        app_tipc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the TIPC. *//**
 *
 * @param[in] msgid     TIPC_ERROR_IND
 * @param[in] param     Pointer to struct tipc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the Application of an error occurred in 
 *  different situations. The error codes are proprietary and defined in prf_types.h. An error may 
 *  occur during attribute discovery or due to application request parameters. Following reception 
 *  of this message, the application will decide the necessary action.
 *
 ****************************************************************************************
 */
int app_tipc_error_ind_handler(ke_msg_id_t const msgid,
                      struct tipc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC error indication status: 0x%X.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Generic message for writing characteristic response status to APP. *//**
 *
 * @param[in] msgid     TIPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct tipc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the Application of a 
 *  received write response. The status and the data from the write response are passed directly 
 *  to Application, which must interpret them based on the request it made. 
 *
 ****************************************************************************************
 */
int app_tipc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC write char response status: 0x%X.\r\n", param->status);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Received Current Time value sent to APP (Read Response or Notification). *//**
 *
 * @param[in] msgid     TIPC_CT_IND
 * @param[in] param     Pointer to struct tipc_ct_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the Application of a received current time 
 *  value. The ind_type parameter informs the application if the value has been notified by the 
 *  Time Client or if it has been received as a read response. 
 *
 ****************************************************************************************
 */
int app_tipc_ct_ind_handler(ke_msg_id_t const msgid,
                      struct tipc_ct_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC Current Time indication(%d): Adjust Reason: %d, Fraction: %d, Day/Week: %d.\r\n", 
        param->ind_type,
        param->ct_val.adjust_reason,
        param->ct_val.exact_time_256.fraction_256,    ///n * (1/256th) of a second
        param->ct_val.exact_time_256.day_date_time.day_of_week
        );

#if QN_DBG_TRACE_MORE
    QPRINTF("Date: %04d:%02d:%02d Time: %02d:%02d:%02d\r\n", 
        param->ct_val.exact_time_256.day_date_time.date_time.year,
        param->ct_val.exact_time_256.day_date_time.date_time.month,
        param->ct_val.exact_time_256.day_date_time.date_time.day,
        param->ct_val.exact_time_256.day_date_time.date_time.hour,
        param->ct_val.exact_time_256.day_date_time.date_time.min,
        param->ct_val.exact_time_256.day_date_time.date_time.sec
        );
#endif
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Read Current Time Notification Configuration sent to APP. *//**
 *
 * @param[in] msgid     TIPC_CT_NTF_CFG_RD_RSP
 * @param[in] param     Pointer to struct tipc_ct_ntf_cfg_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the application that the notification 
 *  configuration value for the Current Time characteristic has been successfully read and to 
 *  provide this value.
 *
 ****************************************************************************************
 */
int app_tipc_ct_ntf_cfg_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_ct_ntf_cfg_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC Read Current Time Notification Configuration: 0x%04X.\r\n", param->ntf_cfg);
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Read Local Time Info value sent to APP. *//**
 *
 * @param[in] msgid     TIPC_LTI_RD_RSP
 * @param[in] param     Pointer to struct tipc_lti_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the application that the LTI characteristic
 *  value has been successfully read and to provide this value.
 * @note
 *
 *  Time Zone Characteristic  UUID: 0x2A0E
 *  Min value : -48 (UTC-12:00), Max value : 56 (UTC+14:00)
 *  -128 : Time zone offset is not known
 *
 *  DST Offset Characteristic  UUID: 0x2A2D
 *  Min value : 0, Max value : 8
 *  255 = DST is not known
 *
 ****************************************************************************************
 */
int app_tipc_lti_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_lti_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC Read Local Time Info, Time Zone(%+d): UTC%d:00, DST Offset: %d.\r\n",
       param->lti_val.time_zone,
       param->lti_val.time_zone/4,
       param->lti_val.dst_offset
       );
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Read Reference Time Info value sent to APP. *//**
 *
 * @param[in] msgid     TIPC_RTI_RD_RSP
 * @param[in] param     Pointer to struct tipc_rti_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the application that the RTI characteristic 
 *  value has been successfully read and to provide this value.
 * @note
 *
 *  Time Source Characteristic  UUID: 0x2A13
 *  Min value : 0, Max value : 6
 *  0 = Unknown
 *  1 = Network Time Protocol
 *  2 = GPS
 *  3 = Radio Time Signal
 *  4 = Manual
 *  5 = Atomic Clock
 *  6 = Cellular Network
 *
 *  Time Accuracy Characteristic  UUID: 0x2A12
 *  Accuracy(drift) of time information in steps of 1/8 of a second(125ms) compared 
 *  to a reference time source. Valid range is from 0 to 253 (0s to 31.5s). A value of
 *  254 means Accuracy is out of range(> 31.5s). A value of 255 means Accuracy is unknown.
 *
 *  Days since last update about Reference Source
 *  Min value: 0, Max value: 254
 *  255 = 255 or more days
 *
 *  Hours since update about Reference Source
 *  Min value: 0, Mac value: 23
 *  255 = 255 or more days (If Days Since Update = 255, then Hours Since Update shall also be set to 255)
 *
 ****************************************************************************************
 */
int app_tipc_rti_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_rti_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    
    QPRINTF("TIPC Read Reference Time Info, Source(%d): ",
        param->rti_val.time_source);
	
#if QN_DBG_TRACE_MORE
	switch (param->rti_val.time_source)
    {
    case 1:
        QPRINTF("Network Time Protocol.\r\n");
        break;
    case 2:
        QPRINTF("GPS.\r\n");
        break;
    case 3:
        QPRINTF("Radio Time Signal.\r\n");
        break;
    case 4:
        QPRINTF("Manual.\r\n");
        break;
    case 5:
        QPRINTF("Atomic Clock.\r\n");
        break;
    case 6:
        QPRINTF("Cellular Network.\r\n");
        break;
    case 0:
    default:
        QPRINTF("Unknown.\r\n");
        break;
    }
    
    QPRINTF("Accuracy: %d, Days Update: %d, Hours Update: %d.\r\n",
        param->rti_val.time_accuracy,
        param->rti_val.days_update,
        param->rti_val.hours_update
    );
#endif

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Read Time With DST value sent to APP. *//**
 *
 * @param[in] msgid     TIPC_TDST_RD_RSP
 * @param[in] param     Pointer struct tipc_tdst_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the application that the TDST 
 *  characteristic value has been successfully read and to provide this value.
 * @note
 *
 *  Time With DST Characteristic Structure - UUID:0x2A11
 *
 ****************************************************************************************
 */
int app_tipc_tdst_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_tdst_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
#if	QN_DBG_TRACE_MORE
    QPRINTF("TIPC Read Time With DST value, DST Offset: %d.\r\n", 
        param->tdst_val.dst_offset);
    QPRINTF("Date: %04d:%02d:%02d Time: %02d:%02d:%02d\r\n", 
        param->tdst_val.date_time.year,
        param->tdst_val.date_time.month,
        param->tdst_val.date_time.day,
        param->tdst_val.date_time.hour,
        param->tdst_val.date_time.min,
        param->tdst_val.date_time.sec
        );
#endif
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}
/*
 ****************************************************************************************
 * @brief Read Time Update State value sent to APP. *//**
 *
 * @param[in] msgid     TIPC_TUS_RD_RSP
 * @param[in] param     Pointer to struct tipc_tus_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the application that the TUS characteristic
 *  value has been successfully read and to provide this value.
 * @note
 *
 *  Time Update State Characteristic Structure - UUID: 0x2A17
 *
 *  The Time Update Status Characteristic exposes the status of the time update
 *  process and the result of the last update in the server. 
 *
 *  Current State
 *  Min value: 0, Max value = 1
 *  0 = Idle
 *  1 = Update Pending
 *
 *  Result
 *  Min value: 0, Max Value: 5
 *  0 = Successful
 *  1 = Canceled
 *  2 = No Connection To Reference
 *  3 = Reference responded with an error
 *  4 = Timeout
 *  5 = Update not attempted after reset
 *
 ****************************************************************************************
 */
int app_tipc_tus_rd_rsp_handler(ke_msg_id_t const msgid,
                      struct tipc_tus_rd_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{

    QPRINTF("TIPC Read Time Update State value, State(%d): , Result(%d).\r\n", 
        param->tus_val.current_state,
        param->tus_val.result
        );

#if QN_DBG_TRACE_MORE   
    QPRINTF("State: ");
    switch (param->tus_val.current_state)
    {
    case TIPS_TIME_UPD_STATE_IDLE:
        QPRINTF("Idle, ");
        break;
    case TIPS_TIME_UPD_STATE_PENDING:
        QPRINTF("Update Pending, ");
        break;
    default:
        QPRINTF("Error, ");
        break;
    }
    QPRINTF("Result: ");
    switch (param->tus_val.result)
    {
    case TIPS_TIME_UPD_RESULT_SUCCESS:
        QPRINTF("Successful.\r\n");
        break;
    case TIPS_TIME_UPD_RESULT_CANCELED:
        QPRINTF("Canceled.\r\n");
        break;
    case TIPS_TIME_UPD_RESULT_NO_CONN:
        QPRINTF("No Connection To Reference.\r\n");
        break;
    case TIPS_TIME_UPD_RESULT_ERROR_RSP:
        QPRINTF("Reference responded with an error.\r\n");
        break;
    case TIPS_TIME_UPD_RESULT_TIMEOUT:
        QPRINTF("Timeout.\r\n");
        break;
    case TIPS_TIME_UPD_NOT_ATTEMPTED:
        QPRINTF("Update not attempted after reset.\r\n");
        break;
    default:
        QPRINTF("Error.\r\n");
        break;
    }
#endif
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     TIPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Time Profile Client Role task
 * has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_tipc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("TIPC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}


#endif

/// @} APP_TIPC_TASK
