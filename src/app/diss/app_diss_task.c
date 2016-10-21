/**
 ****************************************************************************************
 *
 * @file app_diss_task.c
 *
 * @brief Application DISS task implementation
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
 * @addtogroup APP_DISS_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_DIS_SERVER)
#include "app_diss.h"

/// @cond
// Manufacturing name
#define DIS_MANU_NAME_VAL               "NXP"
//Model number
#define DIS_MODEL_NB_VAL                "QN-BLE-B2"
//Serial number
#define DIS_SERIAL_NB_VAL               "1.3.3-LE"
//Firmware Revision string value
#define DIS_FW_REV_VAL                  "6.3.2"
//Hardware Revision string value
#define DIS_HW_REV_VAL                  "0.1.8"
//Software Revision string value
#define DIS_SW_REV_VAL                  "6.2.0"
//System ID
#define DIS_SYS_ID_VAL                  "\x00\x00\x00\xFE\xFF\xBE\x7C\x08"
//IEEE Certif bytes
#define DIS_IEEE_CERTIF_VAL             "\x01\x02\x00\x00\x00\x00"
//PnP ID bytes
#define DIS_PNP_ID_VAL                  "\x03\x04\x00\x00\x00\x00\x00"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_diss_env_tag *app_diss_env = &app_env.diss_ev;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles create database message.      *//**
 *
 * @param[in] msgid     DISS_CREATE_DB_CFM
 * @param[in] param     Pointer to the struct diss_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be called after a database creation. 
 * The status parameter indicates if the DIS has been successfully added or not.
 * Possible values for the status are: ATT_ERR_NO_ERROR and ATT_INSUFF_RESOURCE.
 ****************************************************************************************
 */
int app_diss_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct diss_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR) 
    {
        app_clear_local_service_flag(BLE_DIS_SERVER_BIT);

#if (QN_WORK_MODE == WORK_MODE_SOC)
        if (app_diss_env->features & DIS_MANUFACTURER_NAME_CHAR_SUP)
            app_set_char_val_req(DIS_MANUFACTURER_NAME_CHAR, sizeof(DIS_MANU_NAME_VAL) - 1, DIS_MANU_NAME_VAL);
        if (app_diss_env->features & DIS_MODEL_NB_STR_CHAR_SUP)
            app_set_char_val_req(DIS_MODEL_NB_STR_CHAR, sizeof(DIS_MODEL_NB_VAL) - 1, DIS_MODEL_NB_VAL);
        if (app_diss_env->features & DIS_SERIAL_NB_STR_CHAR_SUP)
            app_set_char_val_req(DIS_SERIAL_NB_STR_CHAR, sizeof(DIS_SERIAL_NB_VAL) - 1, DIS_SERIAL_NB_VAL);
        if (app_diss_env->features & DIS_HARD_REV_STR_CHAR_SUP)
            app_set_char_val_req(DIS_HARD_REV_STR_CHAR, sizeof(DIS_HW_REV_VAL) - 1, DIS_HW_REV_VAL);
        if (app_diss_env->features & DIS_FIRM_REV_STR_CHAR_SUP)
            app_set_char_val_req(DIS_FIRM_REV_STR_CHAR, sizeof(DIS_FW_REV_VAL) - 1, DIS_FW_REV_VAL);
        if (app_diss_env->features & DIS_SW_REV_STR_CHAR_SUP)
            app_set_char_val_req(DIS_SW_REV_STR_CHAR, sizeof(DIS_SW_REV_VAL) - 1, DIS_SW_REV_VAL);
        if (app_diss_env->features & DIS_SYSTEM_ID_CHAR_SUP)
            app_set_char_val_req(DIS_SYSTEM_ID_CHAR, sizeof(DIS_SYS_ID_VAL) - 1, DIS_SYS_ID_VAL);
        if (app_diss_env->features & DIS_IEEE_CHAR_SUP)
            app_set_char_val_req(DIS_IEEE_CHAR, sizeof(DIS_IEEE_CERTIF_VAL) - 1, DIS_IEEE_CERTIF_VAL);
        if (app_diss_env->features & DIS_PNP_ID_CHAR_SUP)
            app_set_char_val_req(DIS_PNP_ID_CHAR, sizeof(DIS_PNP_ID_VAL) - 1, DIS_PNP_ID_VAL);
#endif
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles error indication message.     *//**
 *
 * @param[in] msgid     DISS_ERROR_IND
 * @param[in] param     Pointer to the struct prf_server_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of an occurred error.
 ****************************************************************************************
 */
int app_diss_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

#endif // BLE_DIS_SERVER

/// @} APP_DISS_TASK
