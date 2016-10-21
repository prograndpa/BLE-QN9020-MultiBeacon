/**
 ****************************************************************************************
 *
 * @file app_otas_task.c
 *
 * @brief Application otas implementation
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
 * @addtogroup APP_OTAS_TASK
 * @{
 ****************************************************************************************
 */

 
 /*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
 #include "app_env.h"

#if BLE_OTA_SERVER
/**
 ****************************************************************************************
 * @param[in] msgid     OTAS_TRANSIMIT_STATUS_IND
 * @param[in] param     Pointer to struct otas_transimit_status_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTA
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used by APP to handle OTAS_TRANSIMIT_STATUS_IND msg from otas. 
 * The ota transimition status is in this msg.
 ****************************************************************************************
 */
int app_otas_start_handler(ke_msg_id_t const msgid, struct otas_transimit_status_ind const * param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    app_task_msg_hdl(msgid, param);
    
    return (KE_MSG_CONSUMED);    
}

#endif //BLE_OTA_SERVER
 
/// @} APP_OTAS_TASK
