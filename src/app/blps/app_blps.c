/**
 ****************************************************************************************
 *
 * @file app_blps.c
 *
 * @brief Application BLPS API
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
 * @addtogroup APP_BLPS_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_BP_SENSOR)   
#include "app_blps.h"                 


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
 * @brief Create blood pressure service database        *//**
 *
 * @param[in] features Blood Pressure features used to create database. Possible values are:
 * BLPS_INTM_CUFF_PRESS_SUP
 *
 * @response BLPS_CREATE_DB_CFM
 *
 * @description
 * This function shall be called after system power-on (or after GAP Reset) in order to 
 * create blood pressure profile database. This database will be visible from a peer 
 * device bun not usable until BLPS_ENABLE_REQ message is sent within a BLE connection
 *
 * @note 
 * The Blood Pressure profile requires the presence of three DIS characteristics :
 * Manufacturer Name String, Model Number, System Identifier. It is application's 
 * responsibility to add an instance of the DIS into the database by using the Device 
 * information create database API.
 ****************************************************************************************
 */
void app_blps_create_db(uint8_t features)
{
    struct blps_create_db_req * msg = KE_MSG_ALLOC(BLPS_CREATE_DB_REQ, TASK_BLPS, TASK_APP,
                                                   blps_create_db_req);

    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Enable blood pressure service     *//**
 *
 * @param[in] conhdl Connection handle for which the profile blood pressure sensor role 
 * is enabled.
 * @param[in] sec_lvl Security level required for protection of IAS attributes,
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * Normal connection: Peer device is known and client configuration characteristics shall be restored.
 * Discovery connection: Peer device is unknown and peer collector will manage client configuration
 * characteristics.
 * @param[in] bp_meas_ind_en Value stored for Blood Pressure indication Client Configuration Char.
 * @param[in] interm_cp_ntf_en Value stored for intermediate cuff pressure notification Client 
 * Configuration Char.
 * @param[in] bp_feature Specific blood pressure feature description, Possible values are(See 
 * blp_common.h):
 * - BPS_F_BODY_MVMT_DETECT_NOT_SUPPORTED
 * - BPS_F_BODY_MVMT_DETECT_SUPPORTED
 * - BPS_F_CUFF_FIT_DETECT_NOT_SUPPORTED
 * - BPS_F_CUFF_FIT_DETECT_SUPPORTED
 * - BPS_F_IRREGULAR_PULSE_DETECT_NOT_SUPPORTED
 * - BPS_F_IRREGULAR_PULSE_DETECT_SUPPORTED
 * - BPS_F_PULSE_RATE_RANGE_DETECT_NOT_SUPPORTED
 * - BPS_F_PULSE_RATE_RANGE_DETECT_SUPPORTED
 * - BPS_F_MEAS_POS_DETECT_NOT_SUPPORTED
 * - BPS_F_MEAS_POS_DETECT_SUPPORTED
 * - BPS_F_MULTIPLE_BONDS_NOT_SUPPORTED
 * - BPS_F_MULTIPLE_BONDS_SUPPORTED
 *
 * @response None
 * @description
 * This function is used for enabling the Blood Pressure Sensor role of the Blood Pressure
 * profile. Before calling this function, a BLE connection shall exist with peer device.
 ****************************************************************************************
 */
void app_blps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t bp_meas_ind_en,
                         uint16_t interm_cp_ntf_en, uint16_t bp_feature)
{
    struct blps_enable_req * msg = KE_MSG_ALLOC(BLPS_ENABLE_REQ, TASK_BLPS, TASK_APP,
                                                blps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->bp_meas_ind_en = bp_meas_ind_en;
    msg->interm_cp_ntf_en = interm_cp_ntf_en;
    msg->bp_feature = bp_feature;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send blood pressure measurement value from APP    *//**
 *
 * @param[in] conhdl Connection handle for which the profile blood pressure sensor role is
 * enabled
 * @param[in] flag_interm Own code for differentiating between Blood Pressure Measurement, and
 * Intermediate Cuff Pressure Measurement characteristics
 * @param[in] meas_val Pointer to the struct bps_bp_meas containing Blood Pressure measurement value
 *
 * @response BLPS_MEAS_SEND_CFM or None
 * @description
 * This function is used by the application (which handles the blood pressure device driver and
 * measurements) to send a blood pressure measurement through the blood pressure sensor role.
 * @note 
 *  Message BLPS_CFG_INDNTF_IND will be received as a hint to call this function.
 ****************************************************************************************
 */
void app_blps_pressure_send_req(uint16_t conhdl, uint8_t flag_interm, struct bps_bp_meas *meas_val)
{
    struct blps_meas_send_req * msg = KE_MSG_ALLOC(BLPS_MEAS_SEND_REQ, TASK_BLPS, TASK_APP,
                                                   blps_meas_send_req);

    msg->conhdl = conhdl;
    msg->flag_interm_cp = flag_interm;
    msg->meas_val = *meas_val;
    ke_msg_send(msg);
}

#endif // BLE_BP_SENSOR

/// @} APP_BLPS_API
