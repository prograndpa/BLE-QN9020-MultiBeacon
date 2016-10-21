/**
 ****************************************************************************************
 *
 * @file app_disc_task.c
 *
 * @brief Application DISC task implementation
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
 * @addtogroup APP_DISC_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_DIS_CLIENT
#include "app_disc.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_disc_env_tag *app_disc_env = &app_env.disc_ev[0];
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the DISC. *//**
 *
 * @param[in] msgid     DISC_ENABLE_CFM
 * @param[in] param     Pointer to struct disc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client to either send the discovery results of DIS 
 *  on Server and confirm enabling of the Client role, or to simply confirm enabling of Client role if 
 *  it is a normal connection and the DIS details are already known. An error may have also 
 *  occurred and is signaled.
 *
 ****************************************************************************************
 */
int app_disc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct disc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("DISC enable confirmation status: 0x%x.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);

        app_disc_env[idx].conhdl = param->conhdl;
        app_disc_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     DISC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct disc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Client role to inform the Application of a 
 *  received read response. The status and the data from the read response are passed directly to 
 *  Application, which must interpret them based on the request it made.
 *
 ****************************************************************************************
 */
int app_disc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct disc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
#if QN_DBG_TRACE_MORE
    QPRINTF("DISC read char response status: 0x%x code: 0x%x.\r\n",
        param->status,
        param->char_code);
#endif
    switch (param->char_code)
    {
    case DISC_MANUFACTURER_NAME_CHAR:
        QPRINTF("MANUFACTURER NAME: ");
        QTRACE(param->val, param->val_length, 0, 0);
        break;
    case DISC_MODEL_NB_STR_CHAR:
        QPRINTF("MODEL NB: ");
        QTRACE(param->val, param->val_length, 0, 0);
        break;
    case DISC_SERIAL_NB_STR_CHAR:
        QPRINTF("SERIAL NB: ");
        QTRACE(param->val, param->val_length, 0, 0);
        break;
    case DISC_HARD_REV_STR_CHAR:
        QPRINTF("HARD REV: ");
        QTRACE(param->val, param->val_length, 0, 0);
        break;
    case DISC_FIRM_REV_STR_CHAR:
        QPRINTF("FIRM REV: ");
        QTRACE(param->val, param->val_length, 0, 0);
        break;
    case DISC_SW_REV_STR_CHAR:
        QPRINTF("SW REV: ");
        QTRACE(param->val, param->val_length, 0, 0);
        break;
    case DISC_SYSTEM_ID_CHAR:
        QPRINTF("SYSTEM ID: ");
        if (param->val_length)
        {
            for (uint8_t i = 0; i < param->val_length; i++)
                QPRINTF("%02x", param->val[i]);
        }
        break;
    case DISC_IEEE_CHAR:
        QPRINTF("IEEE CHAR: ");
        if (param->val_length)
        {
            for (uint8_t i = 0; i < param->val_length; i++)
                QPRINTF("%02x", param->val[i]);
        }
        break;
    case DISC_PNP_ID_CHAR:
        QPRINTF("PNP ID: ");
        if (param->val_length)
        {
            for (uint8_t i = 0; i < param->val_length; i++)
                QPRINTF("%02x", param->val[i]);
        }
        break;
    default:
        break;
    }
    QPRINTF("\r\n");
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     DISC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISC
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Device Information Client
 * Role task has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_disc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("DISC disable ind\r\n");

    return (KE_MSG_CONSUMED);
}
#endif

/// @} APP_DISC_TASK
