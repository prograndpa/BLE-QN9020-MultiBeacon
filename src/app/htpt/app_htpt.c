/**
 ****************************************************************************************
 *
 * @file app_htpt.c
 *
 * @brief Application HTPT API
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
 * @addtogroup APP_HTPT_API 
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HT_THERMOM
#include "app_htpt.h"

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
 * @brief Create the health thermometer server service database - at initiation     *//**
 *
 * @param[in] valid_range_min Minimal measurement interval value
 * @param[in] valid_range_max Maximal measurement interval value
 * @param[in] features Indicate if optional features are supported or not. Value of this 
 * parameter shall be set using the following masks:
 * - HTPT_TEMP_TYPE_CHAR_SUP
 * - HTPT_INTERM_TEMP_CHAR_SUP
 * - HTPT_MEAS_INTV_CHAR_SUP
 * - HTPT_MEAS_INTV_IND_SUP
 * - HTPT_MEAS_INTV_WR_SUP
 *
 * @response HTPT_CREATE_DB_CFM
 * @description
 * This function shall be used to add an instance of the Health Thermometer service into
 * the database. This should be done during the initialization phase of the device.
 ****************************************************************************************
 */
void app_htpt_create_db(uint16_t valid_range_min, uint16_t valid_range_max, uint8_t features)
{
    struct htpt_create_db_req * msg = KE_MSG_ALLOC(HTPT_CREATE_DB_REQ, TASK_HTPT, TASK_APP,
                                                   htpt_create_db_req);

    msg->valid_range_min = valid_range_min;
    msg->valid_range_max = valid_range_max;
    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the health thermometer server service - at connection       *//**
 *
 * @param[in] conhdl Connection handle for which the profile Thermometer role is enabled
 * @param[in] sec_lvl Security level required for protection of HTS attributes
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(PRF_CON_DISCOVERY) or normal(PRF_CON_NORMAL)
 * @param[in] temp_meas_ind_en Value stored for Temperature Measurement Client Configuration Characteristic
 * @param[in] interm_temp_ntf_en Value stored for Intermediate Temperature Client Configuration Characteristic
 * @param[in] meas_intv_ind_en Value stored for Measurement Interval Client Configuration Characteristic
 * @param[in] meas_intv Stored Measurement Interval value
 *
 * @response None
 * @description
 * This function is used for enabling the Thermometer role of the Health Thermometer 
 * profile.
 ****************************************************************************************
 */
void app_htpt_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t temp_meas_ind_en,
                         uint16_t interm_temp_ntf_en, uint16_t meas_intv_ind_en, uint16_t meas_intv)
{
    struct htpt_enable_req * msg = KE_MSG_ALLOC(HTPT_ENABLE_REQ, TASK_HTPT, TASK_APP,
                                                htpt_enable_req);
    
    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->temp_meas_ind_en = temp_meas_ind_en;
    msg->interm_temp_ntf_en = interm_temp_ntf_en;
    msg->meas_intv_ind_en = meas_intv_ind_en;
    msg->meas_intv = meas_intv;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send Temperature Measurement value        *//**
 *
 * @param[in] conhdl Connection handle for which the profile Thermometer role is enabled
 * @param[in] temp_meas Pointer to the struct htp_temp_meas containing Temperature Measurement value
 * @param[in] flag_stable_meas Indicate if the temperature measurement is stable: 1 (will
 * be sent using the Temperature Measurement characteristic) or not: 0 (will be sent using the
 * Intermediate Temperature characteristic)
 *
 * @response HTPT_TEMP_SEND_CFM or none
 * @description
 * This function is used by the application (which handles the temperature device
 * driver and measurements) to send a temperature measurement through the Thermometer
 * role.
 * @note Message HTPT_CFG_INDNTF_IND will be received as a hint to call this function.
 ****************************************************************************************
 */
void app_htpt_temp_send(uint16_t conhdl, struct htp_temp_meas *temp_meas, uint8_t flag_stable_meas)
{
    struct htpt_temp_send_req * msg = KE_MSG_ALLOC(HTPT_TEMP_SEND_REQ, TASK_HTPT, TASK_APP,
                                                   htpt_temp_send_req);

    msg->conhdl = conhdl;
    msg->temp_meas = *temp_meas;
    msg->flag_stable_meas = flag_stable_meas;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send Measurement Interval value       *//**
 *
 * @param[in] conhdl Connection handle for which the profile Thermometer role is enabled.
 * @param[in] meas_intv Measurement Interval value.
 *
 * @response None
 * @description
 * This function is used by the application to order the HTPT profile to generate an
 * indication (if enabled) of the Measurement Interval Char. This can be done as the application
 * desires, at each connection, or if the measurement interval value has been modified locally
 * (interface for this is not provided since a normal thermometer would have very few
 * configurable UI elements and configuration should be done through Collector).
 ****************************************************************************************
 */
void app_htpt_measurement_intv_send(uint16_t conhdl, uint16_t meas_intv)
{
    struct htpt_meas_intv_upd_req * msg = KE_MSG_ALLOC(HTPT_MEAS_INTV_UPD_REQ, TASK_HTPT, TASK_APP,
                                                       htpt_meas_intv_upd_req);
    
    msg->conhdl = conhdl;
    msg->meas_intv = meas_intv;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send temperature type         *//**
 *
 * @param[in] value new temperature type value
 *
 * @response None
 * @description
 * This function is used by the application to update the value of the temperature
 * type characteristic.
 ****************************************************************************************
 */
void app_htpt_temp_type_send(uint8_t value)
{
    struct htpt_temp_type_upd_req * msg = KE_MSG_ALLOC(HTPT_TEMP_TYPE_UPD_REQ, TASK_HTPT, TASK_APP,
                                                       htpt_temp_type_upd_req);

    msg->value = value;
    ke_msg_send(msg);
}

#endif // BLE_HT_THERMOM

// @} APP_HTPT_API
