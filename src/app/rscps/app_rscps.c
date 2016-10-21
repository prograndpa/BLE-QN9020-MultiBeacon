/**
 ****************************************************************************************
 *
 * @file app_rscps.c
 *
 * @brief Application RSCPS API
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
 * @addtogroup APP_RSCPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_RSC_SENSOR
#include "app_rscps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create one instance of the Running Speed and Cadence Service in the database - at initiation *//**
 *
 * @param[in] rsc_feature This value is used to decide if the SC Control Point Characteristic 
 * is part of the Running Speed and Cadence service. Not supposed to be modified during 
 * the lifetime of the device
 * @param[in] sensor_loc_supp Indicate if the Sensor Location characteristic is supported.
 * Note that if the Multiple Sensor Location feature is set has supported in the rsc_feature parameter, 
 * the characteristic will be added (mandatory)
 * @param[in] sensor_loc Sensor location, used if the Sensor Location characteristic is added
 * in the database.
 *
 * @response RSCPS_CMP_EVT
 * @description
 * This function shall be called after system power-on in order to create Running Speed and 
 * Cadence Service database. This database will be visible from a peer device but not usable
 * until this service enabled within a BLE connection.
 *
 ****************************************************************************************
 */
void app_rscps_create_db(uint16_t rsc_feature, uint8_t sensor_loc_supp, uint8_t sensor_loc)
{
    struct rscps_create_db_req * msg = KE_MSG_ALLOC(RSCPS_CREATE_DB_REQ, TASK_RSCPS, TASK_APP, rscps_create_db_req);

    msg->rsc_feature = rsc_feature;
    msg->sensor_loc_supp = sensor_loc_supp;
    // Sensor location
    msg->sensor_loc = sensor_loc;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Enable the RSCP Sensor role for the specified connection - at connection *//**
 *
 * @param[in] conhdl Connection handle for which the RSCP Service is enabled
 * @param[in] sec_lvl Security level required for protection of RSCPS attributes
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(PRF_CON_DISCOVERY) or normal(PRF_CON_NORMAL)
 * - Normal connection: Peer device is known(bonded) and Client Configuration Characteristics
 *   values shall be restored
 * - Discovery connection: Peer device is unknown and peer collector will manage Client Configuration 
 *   Characteristics
 * @param[in] rsc_meas_ntf_cfg Value stored for RSC Measurement Characteristic Configuration
 * @param[in] sc_ctnl_pt_ntf_cfg Value stored for SC Control Point Characteristic Configuration
 *
 * @response RSCPS_CMP_EVT
 * @description
 * This function shall be called after the connection with a peer device has been established
 * in order to enable the RSCP Sensor role for the specified connection.
 *
 ****************************************************************************************
 */
void app_rscps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t rsc_meas_ntf_cfg, 
                          uint16_t sc_ctnl_pt_ntf_cfg)
{
    struct rscps_enable_req * msg = KE_MSG_ALLOC(RSCPS_ENABLE_REQ, TASK_RSCPS, TASK_APP, rscps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->rsc_meas_ntf_cfg = rsc_meas_ntf_cfg;
    msg->sc_ctnl_pt_ntf_cfg = sc_ctnl_pt_ntf_cfg;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send the RSC Sensor Measurement value - at connection *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] flags The Flag is used to define which data fields are present in the 
 * characteristic value, possible values are:
 * - RSCP_MEAS_INST_STRIDE_LEN_PRESENT
 * - RSCP_MEAS_TOTAL_DST_MEAS_PRESENT
 * - RSCP_MEAS_WALK_RUN_STATUS
 * @param[in] inst_cad Instantaneous candence
 * @param[in] inst_speed Instantaneous speed
 * @param[in] inst_stride_len Instantaneous stride length
 * @param[in] total_dist Total distance
 *
 * @response RSCPS_CMP_EVT
 * @description
 * This function is used to send a RSC measurement notification to a specific device.
 *
 ****************************************************************************************
 */
void app_rscps_ntf_rsc_meas_req(uint16_t conhdl, uint8_t flags, uint8_t inst_cad, uint16_t inst_speed, 
                                uint16_t inst_stride_len, uint32_t total_dist)
{
    struct rscps_ntf_rsc_meas_cmd * msg = KE_MSG_ALLOC(RSCPS_NTF_RSC_MEAS_CMD, TASK_RSCPS, TASK_APP, rscps_ntf_rsc_meas_cmd);

    msg->conhdl = conhdl;
    msg->flags = flags;
    msg->inst_cad = inst_cad;
    msg->inst_speed = inst_speed;
    msg->inst_stride_len = inst_stride_len;
    msg->total_dist = total_dist;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Application response for the RSCPS_SC_CTNL_PT_REQ_IND message - at connection *//**
 *
 * @param[in] conhdl Connection handle
 * @param[in] status Return the status of control point
 * @param[in] value Values to be sent
 *
 * @response none
 * @description
 * This function is called as a response for the RSCPS_SC_CTNL_PT_REQ_IND message.
 *
 * @note
 * For request supported sensor locations procedure, the field supp_sensor_loc of union rscps_sc_ctnl_pt_cfm_value
 * is a bit mask with one bit for each sensor location. For each sensor location the server device supports,
 * it shall set the corresponding bit in the bit mask.
 ****************************************************************************************
 */
void app_rscps_sc_ctnl_pt_cfm(uint16_t conhdl, uint8_t status, union rscps_sc_ctnl_pt_cfm_value *value)
{
    struct rscps_sc_ctnl_pt_cfm * msg = KE_MSG_ALLOC(RSCPS_SC_CTNL_PT_CFM, TASK_RSCPS, TASK_APP, rscps_sc_ctnl_pt_cfm);

    msg->conhdl = conhdl;
    msg->status = status;
    msg->value = *value;
    ke_msg_send(msg);
}

#endif // BLE_RSC_SENSOR

/// @} APP_RSCPS_API
