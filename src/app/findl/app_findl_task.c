/**
 ****************************************************************************************
 *
 * @file app_findl_task.c
 *
 * @brief Application FMPL task implementation
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
 * @addtogroup APP_FINDL_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_FINDME_LOCATOR
#include "app_findl.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_findl_env_tag *app_findl_env = &app_env.findl_ev[0];
/// @endcond


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the FINDL. *//**
 *
 * @param[in] msgid     FINDL_ENABLE_CFM
 * @param[in] param     Pointer to struct findl_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDL
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Locator to either send the discovery results of 
 *  IAS on Target and confirm enabling of the Locator role, or to simply confirm enabling of Locator 
 *  role if it is a normal connection and the IAS details are already known.  An error may have also 
 *  occurred and is signaled.
 *
 ****************************************************************************************
 */
int app_findl_enable_cfm_handler(ke_msg_id_t const msgid,
                                 struct findl_enable_cfm *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("FINDL enable confirmation status: 0x%x.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR) 
    {
        uint8_t idx = KE_IDX_GET(src_id);
        app_findl_env[idx].conhdl = param->conhdl;
        app_findl_env[idx].enabled = true;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the FINDL. *//**
 *
 * @param[in] msgid     FINDL_ERROR_IND
 * @param[in] param     Pointer to struct findl_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDL
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the application to trigger/stop and alert on the peer 
 *  Target device. The Locator role environment contains the attribute handle for the Alert Level 
 *  Characteristic in the IAS of the Target peer device. This way, a correct request to write this 
 *  attribute to the level requested by the application can be sent. Since the Alert Level 
 *  Characteristic in IAS can only be written using a Write No Response ATT Request, no 
 *  confirmation can be received through the profile. The only confirmation can be observed by 
 *  the user either by noticing and alarm on the Target device or the alarm stopping.
 *
 ****************************************************************************************
 */
int app_findl_error_ind_handler(ke_msg_id_t const msgid,
                                struct findl_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("FINDL error indication status: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable indication to APP. *//**
 *
 * @param[in] msgid     FINDL_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDL
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that the Find Me Profile Locator Role task
 * has been correctly disabled or if an error has occurred during this process.
 *
 ****************************************************************************************
 */
int app_findl_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct prf_client_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("FINDL disable ind.\r\n");

    return (KE_MSG_CONSUMED);
}

#endif

/// @} APP_FINDL_TASK
