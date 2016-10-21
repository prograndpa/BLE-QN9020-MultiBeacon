/**
 ****************************************************************************************
 *
 * @file app_bass.c
 *
 * @brief Application BASS API
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
 * @addtogroup APP_BASS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"
#if BLE_BATT_SERVER
#include "app_bass.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the battery service database - at initiation  *//**
 *
 * @param[in] bas_nb Number of BAS instances to add in the database, the maximal number of
 * BAS that can be added in the DB is defined by the macro BASS_NB_BAS_INSTANCES_MAX
 * @param[in] features Indicate if battery level notify function are supported or not, 
 * possible values are:
 * - BAS_BATT_LVL_NTF_NOT_SUP
 * - BAS_BATT_LVL_NTF_SUP
 *
 * @response BASS_CREATE_DB_CFM
 * @description
 *  Create the battery service database - at initiation.
 ****************************************************************************************
 */
void app_bass_create_db(uint8_t bas_nb, uint8_t *features)
{
    uint8_t i;
    struct bass_create_db_req * msg = KE_MSG_ALLOC(BASS_CREATE_DB_REQ, TASK_BASS, TASK_APP, bass_create_db_req);

    msg->bas_nb = bas_nb;
    // Features of each BAS instance
    for (i = 0; i < bas_nb; i++)
        msg->features[i] = features[i];
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the battery service server - at connection  *//**
 *
 * @param[in] conhdl Connection handle for which the battery service server is enabled
 * @param[in] bas_nb Number of battery service
 * @param[in] sec_lvl Security level required for protection of attributes.
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * @param[in] batt_level_ntf_cfg Pointer to the Battery Level Notification Configurations
 * @param[in] old_batt_lvl Pointer to the Last Battery Level
 * @param[in] current_batt_lvl Pointer to the Current Battery Level
 * @param[in] batt_level_pres_format Pointer to the struct prf_char_pres_fmt containing
 * Battery Level Characteristic Presentation Format
 *
 * @response None
 * @description
 *  Start the battery service server - at connection.
 ****************************************************************************************
 */
void app_bass_enable_req(uint16_t conhdl, uint8_t bas_nb, uint8_t sec_lvl, uint8_t con_type, 
                         uint16_t *batt_level_ntf_cfg,  uint8_t *old_batt_lvl,
                         uint8_t *current_batt_lvl, struct prf_char_pres_fmt *batt_level_pres_format)
{
    struct bass_enable_req * msg = KE_MSG_ALLOC(BASS_ENABLE_REQ, TASK_BASS, TASK_APP, bass_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    
    for (uint8_t i = 0; i < bas_nb; i++)
    {
        msg->batt_level_ntf_cfg[i] = batt_level_ntf_cfg[i];
        msg->old_batt_lvl[i] = old_batt_lvl[i];
        msg->current_batt_lvl[i] = current_batt_lvl[i];
        msg->batt_level_pres_format[i] = batt_level_pres_format[i];
    }
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send the battery level update - at connection  *//**
 *
 * @param[in] conhdl  Connection handle for which the battery service server is enabled
 * @param[in] bas_instance The instances of battery service
 * @param[in] batt_level battery level
 *
 * @response BASS_BATT_LEVEL_UPD_CFM
 * @description
 *  Send the battery level update - at connection.
 ****************************************************************************************
 */
void app_bass_batt_level_upd_req(uint16_t conhdl, uint8_t bas_instance, uint8_t batt_level)
{
    struct bass_batt_level_upd_req * msg = KE_MSG_ALLOC(BASS_BATT_LEVEL_UPD_REQ, TASK_BASS, TASK_APP, bass_batt_level_upd_req);
    
    msg->conhdl = conhdl;
    msg->bas_instance = bas_instance;
    msg->batt_level = batt_level;
    ke_msg_send(msg);
}

#endif // BLE_BATT_SERVER

/// @} APP_BASS_API
