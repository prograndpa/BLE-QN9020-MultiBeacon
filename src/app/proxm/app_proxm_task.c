/**
 ****************************************************************************************
 *
 * @file app_proxm_task.c
 *
 * @brief Application PROXM task implementation
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
 * @addtogroup APP_PROXM_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_PROX_MONITOR
#include "app_proxm.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_proxm_env_tag *app_proxm_env = &app_env.proxm_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the PROXM. *//**
 *
 * @param[in] msgid     PROXM_ENABLE_CFM
 * @param[in] param     Pointer to struct proxm_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Monitor to either send the discovery results of 
 *  LLS, IAS and TPS on Reporter and confirm enabling of the Monitor role, or to simply confirm 
 *  enabling of the Monitor role if it is a normal connection and the LLS, IAS and TPS details are 
 *  already known. 
 * @note
 *  LLS is Mandatory
 *  IAS and TPS are optional
 *
 ****************************************************************************************
 */
int app_proxm_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct proxm_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("PROXM enable confirmation status: 0x%X.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_proxm_env[idx].conhdl = param->conhdl;
        app_proxm_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     PROXM_RD_CHAR_RSP
 * @param[in] param     Pointer to struct proxm_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Monitor role to send the Application the 
 *  characteristic read response data and the status of the read characteristic request. The 
 *  application is in charge of deciphering the data or of the next step if an error is received.
 *
 ****************************************************************************************
 */
int app_proxm_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct proxm_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    uint8_t idx = KE_IDX_GET(src_id);

    QPRINTF("PROXM read char response status: 0x%X.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        if (app_proxm_env[idx].cur_code == 1)
            QPRINTF("Alert Level: ");
        else
            QPRINTF("TX Power Level: ");

        if (app_proxm_env[idx].cur_code == 1)
        {
            switch (param->val)
            {
            case PROXM_ALERT_NONE:
                QPRINTF("None.");
                break;
            case PROXM_ALERT_MILD:
                QPRINTF("Mild.");
                break;
            case PROXM_ALERT_HIGH:
                QPRINTF("High.");
                break;
            default:
                QPRINTF("Error.");
                break;
            }
        }
        else
            QPRINTF("%+d dbm", (int8_t)param->val);
        QPRINTF("\r\n");
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP. *//**
 *
 * @param[in] msgid     PROXM_WR_CHAR_RSP
 * @param[in] param     Pointer to struct proxm_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Monitor role to send the Application the status 
 *  of a characteristic write request, received in a response from TASK_GATT. The application will 
 *  decide what to do if an error is received.
 *
 ****************************************************************************************
 */
int app_proxm_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct proxm_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("PROXM write char response status: 0x%X.\r\n", param->status);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     PROXM_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM
 * @return If the message was consumed or not.
 * @description
 *
 *  This handler is used to inform the application that the Monitor role of the Proximity profile
 *  has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_proxm_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("PROXM disable ind\r\n");

    return (KE_MSG_CONSUMED);
}


#endif

/// @} APP_PROXM_TASK
