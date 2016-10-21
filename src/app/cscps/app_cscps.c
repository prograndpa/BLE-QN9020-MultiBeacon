/**
 ****************************************************************************************
 *
 * @file app_cscps.c
 *
 * @brief Application CSCPS API
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
 * @addtogroup APP_CSCPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_CSC_SENSOR
#include "app_cscps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Create one instance of the Cycling Speed and Cadence Service database - at initiation *//**
 *
 * @param[in] csc_feature This value is used to decide if the SC Control Point Characteristic 
 * is part of the Cycling Speed and Cadence service. Not supposed to be modified during 
 * the lifetime of the device
 * @param[in] sensor_loc_supp Indicate if the Sensor Location characteristic is supported.
 * Note that if the Multiple Sensor Location feature is set has supported in the csc_feature parameter, 
 * the characteristic will be added (mandatory)
 * @param[in] sensor_loc Sensor location
 * 
 * @response CSCPS_CMP_EVT
 * @description
 * This function shall be called after system power-on in order to create Cycling Speed and 
 * Cadence Service database. This database will be visible from a peer device but not usable
 * until this service enabled within a BLE connection.
 *
 ****************************************************************************************
 */
void app_cscps_create_db(uint16_t csc_feature, uint8_t sensor_loc_supp, uint8_t sensor_loc)
{
    struct cscps_create_db_req * msg = KE_MSG_ALLOC(CSCPS_CREATE_DB_REQ, TASK_CSCPS, TASK_APP, cscps_create_db_req);

    msg->csc_feature = csc_feature;
    msg->sensor_loc_supp = sensor_loc_supp;
    // Sensor location
    msg->sensor_loc = sensor_loc;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Enable the CSCP Sensor role for the specified connection - at connection *//**
 *
 * @param[in] conhdl Connection handle for which the CSCP Service is enabled
 * @param[in] sec_lvl Security level required for protection of CSCPS attributes
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(PRF_CON_DISCOVERY) or normal(PRF_CON_NORMAL)
 * - Normal connection: Peer device is known(bonded) and Client Configuration Characteristics
 *   values shall be restored
 * - Discovery connection: Peer device is unknown and peer collector will manage Client Configuration 
 *   Characteristics
 * @param[in] csc_meas_ntf_cfg Value stored for CSC Measurement Characteristic Configuration
 * @param[in] sc_ctnl_pt_ntf_cfg Value stored for SC Control Point Characteristic Configuration
 * @param[in] wheel_rev Stored number of wheel revolution
 *
 * @response CSCPS_CMP_EVT
 * @description
 * This function shall be called after the connection with a peer device has been established
 * in order to enable the CSCP Sensor role for the specified connection.
 *
 ****************************************************************************************
 */
void app_cscps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t csc_meas_ntf_cfg, 
                          uint16_t sc_ctnl_pt_ntf_cfg, uint32_t wheel_rev)
{
    struct cscps_enable_req * msg = KE_MSG_ALLOC(CSCPS_ENABLE_REQ, TASK_CSCPS, TASK_APP, cscps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->csc_meas_ntf_cfg = csc_meas_ntf_cfg;
    msg->sc_ctnl_pt_ntf_cfg = sc_ctnl_pt_ntf_cfg;
    msg->wheel_rev = wheel_rev;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send the CSCP Sensor Measurement value - at connection *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] flags The Flag is used to define which data fields are present in the 
 * characteristic value
 * @param[in] cumul_crank_rev Cumulative Wheel Revolutions value
 * @param[in] last_crank_evt_time the Last Crank Event Time
 * @param[in] last_wheel_evt_time the Last Wheel Event Time
 * @param[in] wheel_rev Wheel Revolution since the last wheel event time
 *
 * @response CSCPS_CMP_EVT
 * @description
 * This function is used to send a CSC measurement notification to a specific device.
 *
 ****************************************************************************************
 */
void app_cscps_ntf_csc_meas_req(uint16_t conhdl, uint8_t flags, uint16_t cumul_crank_rev, uint16_t last_crank_evt_time, 
                                uint16_t last_wheel_evt_time, int16_t wheel_rev)
{
    struct cscps_ntf_csc_meas_cmd * msg = KE_MSG_ALLOC(CSCPS_NTF_CSC_MEAS_CMD, TASK_CSCPS, TASK_APP, cscps_ntf_csc_meas_cmd);

    msg->conhdl = conhdl;
    msg->flags = flags;
    msg->cumul_crank_rev = cumul_crank_rev;
    msg->last_crank_evt_time = last_crank_evt_time;
    msg->last_wheel_evt_time = last_wheel_evt_time;
    msg->wheel_rev = wheel_rev;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Application response for the CSCPS_SC_CTNL_PT_REQ_IND message - at connection *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] status Return the status of control point
 * @param[in] value Values to be sent
 *
 * @response none
 * @description
 * This function is called as a response for the CSCPS_SC_CTNL_PT_REQ_IND message.
 *
 ****************************************************************************************
 */
void app_cscps_sc_ctnl_pt_cfm(uint16_t conhdl, uint8_t status, union cscps_sc_ctnl_pt_cfm_value *value)
{
    struct cscps_sc_ctnl_pt_cfm * msg = KE_MSG_ALLOC(CSCPS_SC_CTNL_PT_CFM, TASK_CSCPS, TASK_APP, cscps_sc_ctnl_pt_cfm);

    msg->conhdl = conhdl;
    msg->status = status;
    msg->value = *value;
    ke_msg_send(msg);
}

#endif // BLE_CSC_SENSOR

/// @} APP_CSCP_API
