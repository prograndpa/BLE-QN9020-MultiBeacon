/**
 ****************************************************************************************
 *
 * @file app_htpt_task.h
 *
 * @brief Application HTPT implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_HTPT_TASK_H_
#define APP_HTPT_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HTPT_TASK Health Thermometer Profile Thermometer Task API
 * @ingroup APP_HTPT
 * @brief Health Thermometer Profile Thermometer Task API
 *
 * Health Thermometer Profile Thermometer Task APIs are used to handle the message from HTPT or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_HT_THERMOM
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

/// @cond

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// The Temperature Type indicates where the temperature was measured.
enum
{
    ARMPIT = 1,
    BODY,
    EAR_LOBE,
    Finger,
    GASTRO_INTESINAL_TRACT, // 5
    MOUTH,
    RECTUM,
    TOE,
    TYMPANUM, // ear drum
};

// temperature flag
enum TEMPERATURE_FLAG_TAG
{
    INTERMEDIATE_MEASUREMENT = 0,
    TEMPERATURE_MEASUREMENT
};

enum app_htpt_state
{
    APP_HTPT_STATE_DISABLED,
    APP_HTPT_STATE_IDLE,
    APP_HTPT_STATE_CONNECTED,
};

struct app_htpt_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    uint8_t features;
    // connection handle
    uint16_t conhdl;
    // htpt meas intv
    uint16_t meas_intv;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_htpt_env_tag *app_htpt_env;

/// @endcond

/*
 ****************************************************************************************
 * @brief 
 *
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the HTPT.    
 *
 ****************************************************************************************
 */
int app_htpt_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct htpt_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the disable service indication from the HTPT.   
 *
 ****************************************************************************************
 */
int app_htpt_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct htpt_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the error indication from the HTPT.      
 *
 ****************************************************************************************
 */
int app_htpt_error_ind_handler(ke_msg_id_t const msgid,
                               struct prf_server_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the send temperature confirm from the HTPT.     
 *
 ****************************************************************************************
 */
int app_htpt_temp_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct htpt_temp_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the send measurement interval indication from the HTPT.     
 *
 ****************************************************************************************
 */
int app_htpt_meas_intv_chg_ind_handler(ke_msg_id_t const msgid,
                                       struct htpt_meas_intv_chg_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the configure ind/ntf indication from the HTPT.   
 *
 ****************************************************************************************
 */
int app_htpt_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct htpt_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
                                   
#endif // BLE_HT_THERMOM

/// @} APP_HTPT_TASK

#endif // APP_HTPT_TASK_H_
