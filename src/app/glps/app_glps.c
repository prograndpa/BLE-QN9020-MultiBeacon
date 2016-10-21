/**
 ****************************************************************************************
 *
 * @file app_glps.c
 *
 * @brief Application GLPS API
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
 * @addtogroup APP_GLPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_GL_SENSOR
#include "app_glps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Create the glucose service database - at initiation     *//**
 *
 * @param[in] start_hdl Glucose Service start handle.Set it to 0 for automatic handle allocation
 * @param[in] meas_ctx_supported Flag used to add or not measurement context in database
 *
 * @response GLPS_CREATE_DB_CFM
 * @description
 * This function shall be called after system power-on (or after GAP Reset) in order to create
 * Glucose profile database. This database will be visible from a peer device but not usable
 * until glps enabled within a BLE connection.
 ****************************************************************************************
 */
void app_glps_create_db(uint16_t start_hdl, uint8_t meas_ctx_supported)
{
    struct glps_create_db_req * msg = KE_MSG_ALLOC(GLPS_CREATE_DB_REQ, TASK_GLPS, TASK_APP, glps_create_db_req);

    msg->start_hdl = start_hdl;
    msg->meas_ctx_supported = meas_ctx_supported;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the glucose profile - at connection     *//**
 * 
 * @param[in] conhdl Connection handle for which the profile Glucose sensor role is enabled
 * @param[in] features Glucose sensor features(see enum glp_srv_feature_flag in glp_common.h)
 * @param[in] sec_lvl Security level required for protection of GLS attributes:
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * @param[in] evt_cfg Glucose sensor event configuration (notification, indication) configured by
 * peer device during another connection (Bonded information)
 * - bit 1: Glucose measurement notifications enabled
 * - bit 2: Glucose measurement context notifications enabled
 * - bit 4: Record Access Control Point (RACP) indications enabled
 *
 * @response GLPS_ENABLE_CFM
 * @description
 * This function is used for enabling the Glucose Sensor role.
 * Before calling this function, a BLE connection shall exist with peer device.
 * Application shall provide connection handle in order to activate the profile.
 ****************************************************************************************
 */
void app_glps_enable_req(uint16_t conhdl, uint16_t features, uint8_t sec_lvl,
                         uint8_t con_type, uint16_t evt_cfg)
{
    struct glps_enable_req * msg = KE_MSG_ALLOC(GLPS_ENABLE_REQ, TASK_GLPS, TASK_APP,
                                                glps_enable_req);

    msg->conhdl = conhdl;
    msg->features = features;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->evt_cfg = evt_cfg;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send Record Access Control Point (RACP) request response - at connection  *//**
 *
 * @param[in] conhdl Connection handle for which the profile Glucose sensor role is enabled
 * @param[in] num_of_record Number of records found(Should be set only if RACP operation code
 * equals GLP_REQ_REP_NUM_OF_STRD_RECS)
 * @param[in] op_code RACP Request operation code(see enum glp_racp_op_code in file glp_common.h)
 * @param[in] status RACP Request operation status code(see enum glp_racp_status in file glp_common.h)
 *
 * @response GLPS_REQ_CMP_EVT
 * @description
 * This function is used by the application to send Record Access Control Point (RACP) 
 * request response.
 * If requested operation is GLP_REQ_REP_NUM_OF_STRD_RECS, number of stored record should be set,
 * else it will be ignored by Glucose sensor role. Status code should be set according to 
 * Glucose profile error code (see enum glp_racp_status in file glp_common.h)
 ****************************************************************************************
 */
void app_glps_racp_rsp_req_send(uint16_t conhdl, uint16_t num_of_record, uint8_t op_code, uint8_t status)
{
    struct glps_racp_rsp_req * msg = KE_MSG_ALLOC(GLPS_RACP_RSP_REQ, TASK_GLPS, TASK_APP,
                                                  glps_racp_rsp_req);

    msg->conhdl = conhdl;
    msg->num_of_record = num_of_record;
    msg->op_code = op_code;
    msg->status = status;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send glucose measurement without following measurement context information *//**
 * 
 * @param[in] conhdl Connection handle for which the profile Glucose sensor role is enabled
 * @param[in] seq_num Measurement Sequence Number
 * @param[in] meas Pointer to the struct glp_meas containing Glucose measurement
 *
 * @response GLPS_REQ_CMP_EVT
 * @description
 * This function is used by the application (which handles the Glucose device driver and measurements)
 * to send a glucose measurement without following measurement context information.
 ****************************************************************************************
 */
void app_glps_meas_without_ctx_req_send(uint16_t conhdl, uint16_t seq_num, struct glp_meas *meas)
{
    struct glps_send_meas_without_ctx_req * msg = KE_MSG_ALLOC(GLPS_SEND_MEAS_WITHOUT_CTX_REQ, TASK_GLPS, TASK_APP,
                                                               glps_send_meas_without_ctx_req);

    msg->conhdl = conhdl;
    msg->seq_num = seq_num;
    msg->meas = *meas;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send glucose measurement with following measurement context information  *//**
 *
 * @param[in] conhdl Connection handle for which the profile Glucose sensor role is enabled
 * @param[in] seq_num Measurement sequence number
 * @param[in] meas Pointer to the struct glp_meas containing Glucose measurement
 * @param[in] ctx Pointer to the struct glp_meas_ctx containing Glucose measurement context structure
 *
 * @response GLPS_REQ_CMP_EVT
 * @description
 * This function is used by the application (which handles the Glucose device driver and measurements)
 * to send a glucose measurement with following measurement context information.
 ****************************************************************************************
 */
void app_glps_meas_with_ctx_req_send(uint16_t conhdl, uint16_t seq_num, struct glp_meas *meas, struct glp_meas_ctx *ctx)
{
    struct glps_send_meas_with_ctx_req * msg = KE_MSG_ALLOC(GLPS_SEND_MEAS_WITH_CTX_REQ, TASK_GLPS, TASK_APP,
                                                            glps_send_meas_with_ctx_req);

    msg->conhdl = conhdl;
    msg->seq_num = seq_num;
    msg->meas = *meas;
    msg->ctx = *ctx;
    ke_msg_send(msg);
}

#endif // BLE_GL_SENSOR

/// @} APP_GLPS_API
