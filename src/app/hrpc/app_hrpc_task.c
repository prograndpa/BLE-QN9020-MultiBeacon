/**
 ****************************************************************************************
 *
 * @file app_hrpc_task.c
 *
 * @brief Application HRPC task implementation
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
 * @addtogroup APP_HRPC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HR_COLLECTOR
#include "app_hrpc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_hrpc_env_tag *app_hrpc_env = &app_env.hrpc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HRPC. *//**
 *
 * @param[in] msgid     HRPC_ENABLE_CFM
 * @param[in] param     Pointer to struct hrpc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector to either send the discovery results of HRS on the Heart Rate 
 *  and confirm enabling of the Collector role, or to simply confirm enabling of Collector role if it is a normal connection 
 *  and the attribute details are already known. 
 *
 ****************************************************************************************
 */
int app_hrpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct hrpc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HRPC enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_hrpc_env[idx].conhdl = param->conhdl;
        app_hrpc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the HRPC. *//**
 *
 * @param[in] msgid     HRPC_ERROR_IND
 * @param[in] param     Pointer to struct hrpc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of an error occurred in different 
 *  situations. The error codes are proprietary and defined in prf_types.h. An error may occur during attribute 
 *  discovery or due to application request parameters. Following reception of this message, the application will decide 
 *  the necessary action
 *
 ****************************************************************************************
 */
int app_hrpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct hrpc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HRPC error indication status: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     HRPC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct hrpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received read response. The 
 *  status and the data from the read response are passed directly to Application, which must interpret them based on 
 *  the request it made. 
 * @note 
 * Response for read Body Sensor Location and Heart Rate Measurement Client Cfg.Desc
 *
 ****************************************************************************************
 */
int app_hrpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hrpc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    QPRINTF("HRPC read char response status: 0x%x.\r\n",
        param->status);

    switch (app_hrpc_env[idx].cur_code)
    {
    case 1:
 #if QN_DBG_TRACE_MORE 
        QPRINTF("HRPC Body Sensor Location is ");
        switch (param->data.data[0])
        {
        case HRS_LOC_CHEST:
            QPRINTF("Chest.\r\n");
            break;
        case HRS_LOC_WRIST:
            QPRINTF("Wrist.\r\n");
            break;
        case HRS_LOC_FINGER:
            QPRINTF("Finger.\r\n");
            break;
        case HRS_LOC_HAND:
            QPRINTF("Hand.\r\n");
            break;
        case HRS_LOC_EAR_LOBE:
            QPRINTF("Ear Lobe.\r\n");
            break;
        case HRS_LOC_FOOT:
            QPRINTF("Foot.\r\n");
            break;
        case HRS_LOC_OTHER:
        default:
            QPRINTF("Other.\r\n");
            break;
        }
#endif
        break;
    default:
        break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP.  *//**
 *
 * @param[in] msgid     HRPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct hrpc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received write response. The 
 *  status and the data from the write response are passed directly to Application, which must interpret them based on 
 *  the request it made.
 *
 ****************************************************************************************
 */
int app_hrpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct hrpc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HRPC write char response status: 0x%x.\r\n", param->status);

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Heart Rate value send to APP. *//**
 *
 * @param[in] msgid     HRPC_HR_MEAS_IND
 * @param[in] param     Pointer to struct hrpc_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received Heart Rate value by 
 *  notification. The application will do what it needs to do with the received measurement. No confirmation of reception 
 *  is needed because the GATT sends it directly to the peer. 
 * @note  
 *
 *  Heart Rate measurement structure refer to struct hrs_hr_meas
 *
 ****************************************************************************************
 */
int app_hrpc_meas_ind_handler(ke_msg_id_t const msgid,
                      struct hrpc_meas_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{

    QPRINTF("Hear Rate Value:%d(bpm), Flag:0x%X, Energy Expended:0x%X.\r\n", 
        param->meas_val.heart_rate,
        param->meas_val.flags,
        (param->meas_val.flags & HRS_FLAG_ENERGY_EXPENDED_PRESENT) ? param->meas_val.energy_expended : 0
        );

#if QN_DBG_TRACE_MORE 
    //One or more RR-Interval values are present. Units: 1/1024 seconds
    if (param->meas_val.flags & HRS_FLAG_RR_INTERVAL_PRESENT)
    {
        QPRINTF("RR-Intervals(%d): ", param->meas_val.nb_rr_interval);
        if (param->meas_val.nb_rr_interval == 0)
            QPRINTF("NULL");
        else
            for (uint8_t i = 0; i < param->meas_val.nb_rr_interval; i++)
                QPRINTF("0x%X ", param->meas_val.rr_intervals[i]);
        QPRINTF("\r\n");
    }
#endif

    //Since Energy Expended is a UINT16, the highest value that can be represented is 65535 kilo Joules.
    //If the maximum value of 65535 kilo Joules is attained (0xFFFF), the field value should remain at 
    //0xFFFF so that the client can be made aware that a reset of the Energy Expended Field is required.
#if 0
    if ((param->meas_val.flags & HRS_FLAG_ENERGY_EXPENDED_PRESENT)
        && (param->meas_val.energy_expended == 0xFFFF))
    {
        app_hrpc_wr_cntl_point_req(1, param->conhdl);
    }
#endif
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//*
 *
 * @param[in] msgid     HRPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Heart Rate Profile Client
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_hrpc_disable_ind_handler(ke_msg_id_t const msgid,
                      struct prf_client_disable_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("HRPC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_HRPC_TASK
