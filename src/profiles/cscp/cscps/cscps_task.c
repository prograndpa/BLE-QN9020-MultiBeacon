/**
 ****************************************************************************************
 *
 * @file cscps_task.c
 *
 * @brief Cycling Speed and Cadence Profile Sensor Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CSCPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_config.h"

#if (BLE_CSC_SENSOR)

#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "cscps.h"
#include "cscps_task.h"
#include "prf_utils.h"
#include <math.h>

/*
 *  RUNNING SPEED AND CADENCE SERVICE ATTRIBUTES
 ****************************************************************************************
 */

/// Cycling Speed and Cadence Sensor Service
static const atts_svc_desc_t cscps_cscs_svc              = ATT_SVC_CYCLING_SPEED_CADENCE;

/// CSC Measurement characteristic value
static const struct atts_char_desc cscps_csc_meas_char   = ATTS_CHAR(ATT_CHAR_PROP_NTF,
                                                                     CSCP_CSCS_CSC_MEAS_CHAR,
                                                                     ATT_CHAR_CSC_MEAS);
/// CSC Feature characteristic value
static const struct atts_char_desc cscps_csc_feat_char   = ATTS_CHAR(ATT_CHAR_PROP_RD,
                                                                     CSCP_CSCS_CSC_FEAT_CHAR,
                                                                     ATT_CHAR_CSC_FEAT);
/// Sensor Location characteristic value
static const struct atts_char_desc cscps_sensor_loc_char = ATTS_CHAR(ATT_CHAR_PROP_RD,
                                                                     CSCP_CSCS_SENSOR_LOC_CHAR,
                                                                     ATT_CHAR_SENSOR_LOC);
/// SC Control Point characteristic value
static const struct atts_char_desc cscps_sc_ctnl_pt_char = ATTS_CHAR(ATT_CHAR_PROP_WR | ATT_CHAR_PROP_IND,
                                                                     CSCP_CSCS_SC_CTNL_PT_CHAR,
                                                                     ATT_CHAR_SC_CNTL_PT);

/// Full CSCPS Database Description - Used to add attributes into the database
static const struct atts_desc cscps_att_db[CSCS_IDX_NB] =
{
    /// Cycling Speed and Cadence Service Declaration
    [CSCS_IDX_SVC]                     =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), sizeof(cscps_cscs_svc),
                                            sizeof(cscps_cscs_svc), (uint8_t *)&cscps_cscs_svc},

    /// CSC Measurement Characteristic Declaration
    [CSCS_IDX_CSC_MEAS_CHAR]           =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(cscps_csc_meas_char),
                                            sizeof(cscps_csc_meas_char), (uint8_t *)&cscps_csc_meas_char},
    /// CSC Measurement Characteristic Value
    [CSCS_IDX_CSC_MEAS_VAL]            =   {ATT_CHAR_CSC_MEAS, PERM(NTF, ENABLE), CSCP_CSC_MEAS_MAX_LEN,
                                            0, NULL},
    /// CSC Measurement Characteristic - Client Characteristic Configuration Descriptor
    [CSCS_IDX_CSC_MEAS_NTF_CFG]        =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE) | PERM(WR, ENABLE), sizeof(uint16_t),
                                            0, NULL},

    /// CSC Feature Characteristic Declaration
    [CSCS_IDX_CSC_FEAT_CHAR]           =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(cscps_csc_feat_char),
                                            sizeof(cscps_csc_feat_char), (uint8_t *)&cscps_csc_feat_char},
    /// CSC Feature Characteristic Value
    [CSCS_IDX_CSC_FEAT_VAL]            =   {ATT_CHAR_CSC_FEAT, PERM(RD, ENABLE), sizeof(uint16_t),
                                            0, NULL},

    /// Sensor Location Characteristic Declaration
    [CSCS_IDX_SENSOR_LOC_CHAR]         =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(cscps_sensor_loc_char),
                                            sizeof(cscps_sensor_loc_char), (uint8_t *)&cscps_sensor_loc_char},
    /// Sensor Location Characteristic Value
    [CSCS_IDX_SENSOR_LOC_VAL]          =   {ATT_CHAR_SENSOR_LOC, PERM(RD, ENABLE), sizeof(uint8_t),
                                            0, NULL},

    /// SC Control Point Characteristic Declaration
    [CSCS_IDX_SC_CTNL_PT_CHAR]         =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(cscps_sc_ctnl_pt_char),
                                            sizeof(cscps_sc_ctnl_pt_char), (uint8_t *)&cscps_sc_ctnl_pt_char},
    /// SC Control Point Characteristic Value - The response has the maximal length
    [CSCS_IDX_SC_CTNL_PT_VAL]          =   {ATT_CHAR_SC_CNTL_PT, PERM(WR, ENABLE) | PERM(IND, ENABLE), CSCP_SC_CNTL_PT_RSP_MAX_LEN,
                                            0, NULL},
    /// SC Control Point Characteristic - Client Characteristic Configuration Descriptor
    [CSCS_IDX_SC_CTNL_PT_NTF_CFG]      =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE) | PERM(WR, ENABLE), sizeof(uint16_t),
                                            0, NULL},
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CSCPS_CREATE_DB_REQ message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int cscps_create_db_req_handler(ke_msg_id_t const msgid,
                                       struct cscps_create_db_req *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Service Configuration Flag
    uint16_t cfg_flag = CSCPS_MANDATORY_MASK;
    // Database Creation Status
    uint8_t status;

    // Check if a Cycling Speed and Cadence service has already been added in the database
    if (ke_state_get(dest_id) == CSCPS_DISABLED)
    {
        /*
         * Check if the Sensor Location characteristic shall be added.
         * Mandatory if the Multiple Sensor Location feature is supported, otherwise optional.
         */
        if ((param->sensor_loc_supp == CSCPS_SENSOR_LOC_SUPP) ||
            (CSCPS_IS_FEATURE_SUPPORTED(param->csc_feature, CSCP_FEAT_MULT_SENSOR_LOC_SUPP)))
        {
            cfg_flag |= CSCPS_SENSOR_LOC_MASK;
        }

        /*
         * Check if the SC Control Point characteristic shall be added
         * Mandatory if at least one SC Control Point procedure is supported, otherwise excluded.
         */
        if (CSCPS_IS_FEATURE_SUPPORTED(param->csc_feature, CSCP_FEAT_WHEEL_REV_DATA_SUPP) ||
            CSCPS_IS_FEATURE_SUPPORTED(param->csc_feature, CSCP_FEAT_MULT_SENSOR_LOC_SUPP))
        {
            cfg_flag |= CSCPS_SC_CTNL_PT_MASK;
        }

        // Add service in the database
        status = atts_svc_create_db(&cscps_env.shdl, (uint8_t *)&cfg_flag, CSCS_IDX_NB,
                                    &cscps_env.hdl_offset[0], dest_id, &cscps_att_db[0]);

        // Check if an error has occured
        if (status == ATT_ERR_NO_ERROR)
        {
            // Force the unused bits of the CSC Feature Char value to 0
            param->csc_feature &= CSCP_FEAT_ALL_SUPP;

            // Store the value in the environment
            cscps_env.prf_cfg = param->csc_feature;

            // Set the value of the CSC Feature characteristic
            attsdb_att_set_value(cscps_env.shdl + CSCS_IDX_CSC_FEAT_VAL, sizeof(uint16_t),
                                 (uint8_t *)&param->csc_feature);

            // If the Sensor Location characteristic has been added, set the value
            if (cscps_env.hdl_offset[CSCP_CSCS_SENSOR_LOC_CHAR] != 0x00)
            {
                // Check if the provided value is valid, else force the location to Other
                if (param->sensor_loc >= CSCP_LOC_MAX)
                {
                    param->sensor_loc = CSCP_LOC_OTHER;
                }

                // Set the value of the Sensor Location characteristic
                attsdb_att_set_value(cscps_env.shdl + CSCS_IDX_SENSOR_LOC_VAL, sizeof(uint8_t),
                                     (uint8_t *)&param->sensor_loc);
            }

            // Disable CSCS
            attsdb_svc_set_permission(cscps_env.shdl, PERM(SVC, DISABLE));

            // If we are here, database has been fulfilled with success, go to idle state
            ke_state_set(TASK_CSCPS, CSCPS_IDLE);
        }
    }
    else
    {
        // Request is disallowed, a CSCS has already been added
        status = PRF_ERR_REQ_DISALLOWED;
    }

    // Send complete event message to the application
    cscps_send_cmp_evt(TASK_CSCPS, src_id, GAP_INVALID_CONHDL, CSCPS_CREATE_DB_OP_CODE, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CSCPS_ENABLE_REQ message.
 * @param[in] msgid Id of the message received
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int cscps_enable_req_handler(ke_msg_id_t const msgid,
                                    struct cscps_enable_req *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_OK;

    // Check the current state of the task and if the provided connection handle is valid
    if ((ke_state_get(TASK_CSCPS) == CSCPS_IDLE) &&
        (gap_get_rec_idx(param->conhdl) != GAP_INVALID_CONIDX))
    {
        // Fill the Connection Information structure
        cscps_env.con_info.conhdl = param->conhdl;
        cscps_env.con_info.prf_id = dest_id;
        cscps_env.con_info.appid  = src_id;

        // Reset the store Client Characteristic Configuration status
        cscps_env.prf_cfg &= CSCP_FEAT_ALL_SUPP;

        // Set the CSC Measurement Char. Client Characteristic Configuration value
        if (param->con_type == PRF_CON_DISCOVERY)
        {
            // Force the configuration to 0
            param->csc_meas_ntf_cfg   = PRF_CLI_STOP_NTFIND;
        }
        else
        {
            // Check the provided value
            if (param->csc_meas_ntf_cfg == PRF_CLI_START_NTF)
            {
                // Store the status
                CSCPS_ENABLE_NTFIND(CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF);
            }
            else if (param->csc_meas_ntf_cfg != PRF_CLI_STOP_NTFIND)
            {
                status = PRF_ERR_INVALID_PARAM;
            }
        }

        // If supported, set the SC Control Point Char. Client Characteristic Configuration value
        if ((status == PRF_ERR_OK) &&
            (cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] != 0x00))
        {
            if (param->con_type == PRF_CON_DISCOVERY)
            {
                // Force the configuration to 0
                param->sc_ctnl_pt_ntf_cfg   = PRF_CLI_STOP_NTFIND;
            }
            else
            {
                // Check the provided value
                if (param->sc_ctnl_pt_ntf_cfg == PRF_CLI_START_IND)
                {
                    // Store the status
                    CSCPS_ENABLE_NTFIND(CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND);
                }
                else if (param->sc_ctnl_pt_ntf_cfg != PRF_CLI_STOP_NTFIND)
                {
                    status = PRF_ERR_INVALID_PARAM;
                }
            }
        }

        if (status == PRF_ERR_OK)
        {
            // Set the CSC Measurement Char. Client Characteristic Configuration value in the database
            attsdb_att_set_value(cscps_env.shdl + CSCS_IDX_CSC_MEAS_NTF_CFG,
                                 sizeof(uint16_t), (uint8_t *)&param->csc_meas_ntf_cfg);

            if (cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] != 0x00)
            {
                // Set the SC Control Point Char. Client Characteristic Configuration value in the database
                attsdb_att_set_value(cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] + 2,
                                     sizeof(uint16_t), (uint8_t *)&param->sc_ctnl_pt_ntf_cfg);
            }

            // Store the provided cumulative wheel revolution value
            cscps_env.wheel_revol = param->wheel_rev;

            // Enable CSCS
            attsdb_svc_set_permission(cscps_env.shdl, param->sec_lvl);

            // Go to Connected State
            ke_state_set(dest_id, CSCPS_CONNECTED);
        }
    }
    else
    {
        // The request is disallowed (profile already enabled for this connection, or not enough memory, ...)
        status = PRF_ERR_REQ_DISALLOWED;
    }

    // Send response to application
    cscps_send_cmp_evt(dest_id, src_id, param->conhdl, CSCPS_ENABLE_OP_CODE, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CSCPS_NTF_CSC_MEAS_CMD message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int cscps_ntf_csc_meas_cmd_handler(ke_msg_id_t const msgid,
                                          struct cscps_ntf_csc_meas_cmd *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;
    // State
    ke_state_t state   = ke_state_get(TASK_CSCPS);

    // State shall be Connected or Busy
    if (state >= CSCPS_CONNECTED)
    {
        // Status
        uint8_t status     = PRF_ERR_OK;

        // Packed Measurement
        uint8_t pckd_meas[CSCP_CSC_MEAS_MAX_LEN];
        // Packed Measurement length
        uint8_t pckd_meas_len = CSCP_CSC_MEAS_MIN_LEN;

        do
        {
            // Check the connection handle
            if (param->conhdl != cscps_env.con_info.conhdl)
            {
                status = PRF_ERR_INVALID_PARAM;
                break;
            }

            // Check if sending of notifications has been enabled
            if (!CSCPS_IS_NTFIND_ENABLED(CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF))
            {
                status = PRF_ERR_NTF_DISABLED;
                break;
            }

            // Check the current state of the task
            if (state == CSCPS_BUSY)
            {
                // Keep the message for later, status will be PRF_ERR_OK
                msg_status = KE_MSG_SAVED;
                break;
            }

            ASSERT_ERR(cscps_env.operation == CSCPS_RESERVED_OP_CODE);

            // Force the unused bits of the flag value to 0
            param->flags &= CSCP_MEAS_ALL_PRESENT;

            // Check the provided flags value
            if (!CSCPS_IS_FEATURE_SUPPORTED(cscps_env.prf_cfg, CSCP_FEAT_WHEEL_REV_DATA_SUPP) &&
                CSCPS_IS_PRESENT(param->flags, CSCP_MEAS_WHEEL_REV_DATA_PRESENT))
            {
                // Force Wheel Revolution Data to No (Not supported)
                param->flags &= ~CSCP_MEAS_WHEEL_REV_DATA_PRESENT;
            }

            if (!CSCPS_IS_FEATURE_SUPPORTED(cscps_env.prf_cfg, CSCP_FEAT_CRANK_REV_DATA_SUPP) &&
                CSCPS_IS_PRESENT(param->flags, CSCP_MEAS_CRANK_REV_DATA_PRESENT))
            {
                // Force Crank Revolution Data Present to No (Not supported)
                param->flags &= ~CSCP_MEAS_CRANK_REV_DATA_PRESENT;
            }

            // Flag value
            pckd_meas[0] = param->flags;

            // Cumulative Wheel Resolutions
            // Last Wheel Event Time
            if (CSCPS_IS_PRESENT(param->flags, CSCP_MEAS_WHEEL_REV_DATA_PRESENT))
            {
                // Update the cumulative wheel revolutions value stored in the environment
                if (param->wheel_rev < 0)
                {
                    // The value shall not decrement below zero
                    if (fabs(param->wheel_rev) > cscps_env.wheel_revol)
                    {
                        cscps_env.wheel_revol = 0;
                    }
                    else
                    {
                        cscps_env.wheel_revol += param->wheel_rev;
                    }
                }
                else
                {
                    cscps_env.wheel_revol += param->wheel_rev;
                }

                // Cumulative Wheel Resolutions
                co_write32p(&pckd_meas[pckd_meas_len], cscps_env.wheel_revol);
                pckd_meas_len += 4;

                // Last Wheel Event Time
                co_write16p(&pckd_meas[pckd_meas_len], param->last_wheel_evt_time);
                pckd_meas_len += 2;
            }

            // Cumulative Crank Revolutions
            // Last Crank Event Time
            if (CSCPS_IS_PRESENT(param->flags, CSCP_MEAS_CRANK_REV_DATA_PRESENT))
            {
                // Cumulative Crank Revolutions
                co_write16p(&pckd_meas[pckd_meas_len], param->cumul_crank_rev);
                pckd_meas_len += 2;

                // Last Crank Event Time
                co_write16p(&pckd_meas[pckd_meas_len], param->last_crank_evt_time);
                pckd_meas_len += 2;
            }

            // Set the value in the database
            attsdb_att_set_value(cscps_env.shdl + CSCS_IDX_CSC_MEAS_VAL, pckd_meas_len,
                                 (uint8_t *)&pckd_meas[0]);

            // Send the notification
            struct gatt_notify_req *ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ,
                                                       TASK_GATT, TASK_CSCPS,
                                                       gatt_notify_req);

            ntf->conhdl  = cscps_env.con_info.conhdl;
            ntf->charhdl = cscps_env.shdl + CSCS_IDX_CSC_MEAS_VAL;

            ke_msg_send(ntf);

            // Configure the environment
            cscps_env.operation = CSCPS_SEND_CSC_MEAS_OP_CODE;
            // Go to busy state
            ke_state_set(TASK_CSCPS, CSCPS_BUSY);
        } while (0);

        if (status != PRF_ERR_OK)
        {
            // Send response to application
            cscps_send_cmp_evt(cscps_env.con_info.prf_id, cscps_env.con_info.appid, cscps_env.con_info.conhdl,
                               CSCPS_SEND_CSC_MEAS_OP_CODE, status);
        }
    }
    else
    {
        // The profile has not been enabled for a connection or the database has not been created
        cscps_send_cmp_evt(dest_id, src_id, param->conhdl, CSCPS_SEND_CSC_MEAS_OP_CODE, PRF_ERR_REQ_DISALLOWED);
    }

    return (int)msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref CSCPS_SC_CTNL_PT_CFM message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int cscps_sc_ctnl_pt_cfm_handler(ke_msg_id_t const msgid,
                                          struct cscps_sc_ctnl_pt_cfm *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // SC Control Point Response
    uint8_t rsp[CSCP_SC_CNTL_PT_RSP_MAX_LEN];
    // Response Length (At least 3)
    uint8_t rsp_len = CSCP_SC_CNTL_PT_RSP_MIN_LEN;

    do
    {
        // Check the current operation
        if (cscps_env.operation < CSCPS_CTNL_PT_CUMUL_VAL_OP_CODE)
        {
            // The confirmation has been sent without request indication, ignore
            break;
        }

        // The SC Control Point Characteristic is supported if we are here
        ASSERT_ERR(cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] != 0x00);

        // Set the operation code (Response Code)
        rsp[0] = CSCP_CTNL_PT_RSP_CODE;
        // Set the response value
        rsp[2] = (param->status == PRF_ERR_OK) ? CSCP_CTNL_PT_RESP_SUCCESS : CSCP_CTNL_PT_RESP_FAILED;

        switch (cscps_env.operation)
        {
            // Set cumulative value
            case (CSCPS_CTNL_PT_CUMUL_VAL_OP_CODE):
            {
                // Set the request operation code
                rsp[1] = CSCP_CTNL_PT_OP_SET_CUMUL_VAL;

                // Store the new value in the environment
                cscps_env.wheel_revol = param->value.cumul_wheel_rev;
            } break;

            // Update Sensor Location
            case (CSCPS_CTNL_PT_UPD_LOC_OP_CODE):
            {
                // Set the request operation code
                rsp[1] = CSCP_CTNL_PT_OP_UPD_LOC;

                if (param->status == PRF_ERR_OK)
                {
                    // The Sensor Location Characteristic is supported if we are here
                    ASSERT_ERR(cscps_env.hdl_offset[CSCP_CSCS_SENSOR_LOC_CHAR] != 0x00);

                    // If operation is successful, update the value in the database
                    attsdb_att_set_value(cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SENSOR_LOC_CHAR] + 1,
                                         sizeof(uint8_t), (uint8_t *)&param->value.sensor_loc);
                }
            } break;

            case (CSCPS_CTNL_PT_SUPP_LOC_OP_CODE):
            {
                // Set the request operation code
                rsp[1] = CSCP_CTNL_PT_OP_REQ_SUPP_LOC;

                if (param->status == PRF_ERR_OK)
                {
                    // Counter
                    uint8_t counter;

                    // Set the list of supported location
                    for (counter = 0; counter < CSCP_LOC_MAX; counter++)
                    {
                        if (((param->value.supp_sensor_loc >> counter) & 0x0001) == 0x0001)
                        {
                            rsp[rsp_len] = counter;
                            rsp_len++;
                        }
                    }
                }
            } break;

            default:
            {
                ASSERT_ERR(0);
            } break;
        }

        // Set the value in the database - If we are here the handle is valid
        attsdb_att_set_value(cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] + 1,
                             rsp_len, (uint8_t *)&rsp);

        // Send the response indication to the peer device
        struct gatt_indicate_req *ind = KE_MSG_ALLOC(GATT_INDICATE_REQ,
                                                     TASK_GATT, cscps_env.con_info.prf_id,
                                                     gatt_indicate_req);

        ind->conhdl  = cscps_env.con_info.conhdl;
        ind->charhdl = cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] + 1;

        ke_msg_send(ind);
    } while (0);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gatt_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Check the connection handle
    if (ke_state_get(TASK_CSCPS) >= CSCPS_CONNECTED)
    {
        // CSC Measurement Characteristic, Client Characteristic Configuration Descriptor
        if (param->handle == (cscps_env.shdl + CSCS_IDX_CSC_MEAS_NTF_CFG))
        {
            uint16_t ntf_cfg;
            // Status
            uint8_t status = PRF_CCCD_IMPR_CONFIGURED;

            // Get the value
            co_write16p(&ntf_cfg, param->value[0]);

            // Check if the value is correct
            if (ntf_cfg <= PRF_CLI_START_NTF)
            {
                status = PRF_ERR_OK;

                // Set the value in the database
                attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&ntf_cfg);

                // Save the new configuration in the environment
                if (ntf_cfg == PRF_CLI_STOP_NTFIND)
                {
                    CSCPS_DISABLE_NTFIND(CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF);
                }
                else    // ntf_cfg == PRF_CLI_START_NTF
                {
                    CSCPS_ENABLE_NTFIND(CSCP_PRF_CFG_FLAG_CSC_MEAS_NTF);
                }

                // Inform the HL about the new configuration
                struct cscps_ntf_ind_cfg_ind *ind = KE_MSG_ALLOC(CSCPS_NTF_IND_CFG_IND,
                                                                 cscps_env.con_info.appid, cscps_env.con_info.prf_id,
                                                                 cscps_ntf_ind_cfg_ind);

                ind->conhdl    = param->conhdl;
                ind->char_code = CSCP_CSCS_CSC_MEAS_CHAR;
                ind->ntf_cfg   = ntf_cfg;

                ke_msg_send(ind);
            }
            // else status is PRF_CCCD_IMPR_CONFIGURED

            // Send the write response to the peer device
            atts_write_rsp_send(param->conhdl, param->handle, status);
        }
        else        // Should be the SC Control Point Characteristic
        {
            ASSERT_ERR(cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] != 0x00);

            // SC Control Point, Client Characteristic Configuration Descriptor
            if (param->handle == (cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] + 2))
            {
                uint16_t ntf_cfg;
                // Status
                uint8_t status = PRF_CCCD_IMPR_CONFIGURED;

                // Get the value
                co_write16p(&ntf_cfg, param->value[0]);

                // Check if the value is correct
                if ((ntf_cfg == PRF_CLI_STOP_NTFIND) || (ntf_cfg == PRF_CLI_START_IND))
                {
                    status = PRF_ERR_OK;

                    // Set the value in the database
                    attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&ntf_cfg);

                    // Save the new configuration in the environment
                    if (ntf_cfg == PRF_CLI_STOP_NTFIND)
                    {
                        CSCPS_DISABLE_NTFIND(CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND);
                    }
                    else    // ntf_cfg == PRF_CLI_START_IND
                    {
                        CSCPS_ENABLE_NTFIND(CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND);
                    }

                    // Inform the HL about the new configuration
                    struct cscps_ntf_ind_cfg_ind *ind = KE_MSG_ALLOC(CSCPS_NTF_IND_CFG_IND,
                                                                     cscps_env.con_info.appid, cscps_env.con_info.prf_id,
                                                                     cscps_ntf_ind_cfg_ind);

                    ind->conhdl    = param->conhdl;
                    ind->char_code = CSCP_CSCS_SC_CTNL_PT_CHAR;
                    ind->ntf_cfg   = ntf_cfg;

                    ke_msg_send(ind);
                }
                // else status is PRF_CCCD_IMPR_CONFIGURED

                // Send the write response to the peer device
                atts_write_rsp_send(param->conhdl, param->handle, status);
            }
            // SC Control Point Characteristic
            else if (param->handle == (cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] + 1))
            {
                // Write Response Status
                uint8_t wr_status  = PRF_ERR_OK;
                // Indication Status
                uint8_t ind_status = CSCP_CTNL_PT_RESP_NOT_SUPP;

                do
                {
                    // Check if sending of indications has been enabled
                    if (!CSCPS_IS_NTFIND_ENABLED(CSCP_PRF_CFG_FLAG_SC_CTNL_PT_IND))
                    {
                        // CCC improperly configured
                        wr_status = PRF_CCCD_IMPR_CONFIGURED;

                        break;
                    }

                    if (cscps_env.operation >= CSCPS_CTNL_PT_CUMUL_VAL_OP_CODE)
                    {
                        // A procedure is already in progress
                        wr_status = CSCP_ERROR_PROC_IN_PROGRESS;

                        break;
                    }

                    if (cscps_env.operation == CSCPS_SEND_CSC_MEAS_OP_CODE)
                    {
                        // Keep the message until the end of the current procedure
                        msg_status = KE_MSG_SAVED;

                        break;
                    }

                    // Allocate a request indication message for the application
                    struct cscps_sc_ctnl_pt_req_ind *req_ind = KE_MSG_ALLOC(CSCPS_SC_CTNL_PT_REQ_IND,
                                                                            cscps_env.con_info.appid, cscps_env.con_info.prf_id,
                                                                            cscps_sc_ctnl_pt_req_ind);

                    // Connection Handle
                    req_ind->conhdl    = param->conhdl;
                    // Operation Code
                    req_ind->op_code   = param->value[0];

                    // Operation Code
                    switch(param->value[0])
                    {
                        // Set Cumulative value
                        case (CSCP_CTNL_PT_OP_SET_CUMUL_VAL):
                        {
                            // Check if the Wheel Revolution Data feature is supported
                            if (CSCPS_IS_FEATURE_SUPPORTED(cscps_env.prf_cfg, CSCP_FEAT_WHEEL_REV_DATA_SUPP))
                            {
                                // The request can be handled
                                ind_status = PRF_ERR_OK;

                                cscps_env.operation = CSCPS_CTNL_PT_CUMUL_VAL_OP_CODE;

                                // Cumulative value
                                req_ind->value.cumul_value = co_read32p(&param->value[1]);
                            }
                        } break;

                        // Update sensor location
                        case (CSCP_CTNL_PT_OP_UPD_LOC):
                        {
                            // Check if the Multiple Sensor Location feature is supported
                            if (CSCPS_IS_FEATURE_SUPPORTED(cscps_env.prf_cfg, CSCP_FEAT_MULT_SENSOR_LOC_SUPP))
                            {
                                // Check the sensor location value
                                if (param->value[1] < CSCP_LOC_MAX)
                                {
                                    // The request can be handled
                                    ind_status = PRF_ERR_OK;

                                    cscps_env.operation = CSCPS_CTNL_PT_UPD_LOC_OP_CODE;

                                    // Sensor Location
                                    req_ind->value.sensor_loc = param->value[1];
                                }
                                else
                                {
                                    // Provided parameter in not within the defined range
                                    ind_status = CSCP_CTNL_PT_RESP_INV_PARAM;
                                }
                            }
                        } break;

                        // Request supported sensor locations
                        case (CSCP_CTNL_PT_OP_REQ_SUPP_LOC):
                        {
                            // Check if the Multiple Sensor Location feature is supported
                            if (CSCPS_IS_FEATURE_SUPPORTED(cscps_env.prf_cfg, CSCP_FEAT_MULT_SENSOR_LOC_SUPP))
                            {
                                // The request can be handled
                                ind_status = PRF_ERR_OK;

                                cscps_env.operation = CSCPS_CTNL_PT_SUPP_LOC_OP_CODE;
                            }
                        } break;

                        default:
                        {
                            // Operation Code is invalid, status is already CSCP_CTNL_PT_RESP_NOT_SUPP
                        } break;
                    }

                    // Go to the Busy status
                    ke_state_set(TASK_CSCPS, CSCPS_BUSY);

                    // If no error raised, inform the application about the request
                    if (ind_status == PRF_ERR_OK)
                    {
                        // Send the request indication to the application
                        ke_msg_send(req_ind);
                    }
                    else
                    {
                        // Free the allocated message
                        ke_msg_free(ke_param2msg(req_ind));

                        // Send an error indication
                        cscps_send_rsp_ind(param->handle, param->value[0], ind_status);

                        cscps_env.operation = CSCPS_CTNL_ERR_IND_OP_CODE;
                    }
                } while (0);

                // Send the write response to the peer device
                atts_write_rsp_send(param->conhdl, param->handle, wr_status);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
    }
    // else drop the message

    return (int)msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_HANDLE_VALUE_CFM message meaning that indication
 * has been correctly sent to peer device.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_handle_value_cfm_handler(ke_msg_id_t const msgid,
                                        struct gatt_handle_value_cfm const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // Check if a connection exists
    if (ke_state_get(TASK_CSCPS) >= CSCPS_CONNECTED)
    {
        ASSERT_ERR(ke_state_get(TASK_CSCPS) == CSCPS_BUSY);
        ASSERT_ERR(cscps_env.operation >= CSCPS_CTNL_PT_CUMUL_VAL_OP_CODE);
        ASSERT_ERR(param->handle == cscps_env.shdl + cscps_env.hdl_offset[CSCP_CSCS_SC_CTNL_PT_CHAR] + 1);

        // Inform the application that a procedure has been completed
        cscps_send_cmp_evt(cscps_env.con_info.prf_id, cscps_env.con_info.appid, cscps_env.con_info.conhdl,
                           cscps_env.operation, param->status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device)
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid,
                                       struct gatt_notify_cmp_evt const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
#if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)  
    if(param->status == GATT_NOTIFY_GET_DATA)
        return (KE_MSG_CONSUMED);        
#endif

    // Check if a connection exists
    if (ke_state_get(TASK_CSCPS) >= CSCPS_CONNECTED)
    {
        ASSERT_ERR(ke_state_get(TASK_CSCPS) == CSCPS_BUSY);
        ASSERT_ERR(cscps_env.operation == CSCPS_SEND_CSC_MEAS_OP_CODE);
        ASSERT_ERR(param->handle == cscps_env.shdl + CSCS_IDX_CSC_MEAS_VAL);

        // Inform the application that a procedure has been completed
        cscps_send_cmp_evt(cscps_env.con_info.prf_id, cscps_env.con_info.appid, cscps_env.con_info.conhdl,
                           CSCPS_SEND_CSC_MEAS_OP_CODE, param->status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to CSCPS.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gap_discon_cmp_evt_handler(ke_msg_id_t const msgid,
                                        struct gap_discon_cmp_evt const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // Check if a connection exists
    if (ke_state_get(TASK_CSCPS) >= CSCPS_CONNECTED)
    {
        // Check Connection Handle
        if (param->conhdl == cscps_env.con_info.conhdl)
        {
            cscps_disable();
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
const struct ke_msg_handler cscps_default_state[] =
{
    {CSCPS_CREATE_DB_REQ,           (ke_msg_func_t)cscps_create_db_req_handler},
    {CSCPS_ENABLE_REQ,              (ke_msg_func_t)cscps_enable_req_handler},
    {CSCPS_NTF_CSC_MEAS_CMD,        (ke_msg_func_t)cscps_ntf_csc_meas_cmd_handler},
    {CSCPS_SC_CTNL_PT_CFM,          (ke_msg_func_t)cscps_sc_ctnl_pt_cfm_handler},

    {GATT_NOTIFY_CMP_EVT,           (ke_msg_func_t)gatt_notify_cmp_evt_handler},
    {GATT_HANDLE_VALUE_CFM,         (ke_msg_func_t)gatt_handle_value_cfm_handler},
    {GATT_WRITE_CMD_IND,            (ke_msg_func_t)gatt_write_cmd_ind_handler},

    {GAP_DISCON_CMP_EVT,            (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler cscps_state_handler[CSCPS_STATE_MAX] =
{
    [CSCPS_DISABLED]    = KE_STATE_HANDLER_NONE,
    [CSCPS_IDLE]        = KE_STATE_HANDLER_NONE,
    [CSCPS_CONNECTED]   = KE_STATE_HANDLER_NONE,
    [CSCPS_BUSY]        = KE_STATE_HANDLER_NONE,
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler cscps_default_handler = KE_STATE_HANDLER(cscps_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t cscps_state[CSCPS_IDX_MAX];

// Register CSCPS TASK into kernel
void task_cscps_desc_register(void)
{
    struct ke_task_desc task_cscps_desc;

    task_cscps_desc.state_handler = cscps_state_handler;
    task_cscps_desc.default_handler = &cscps_default_handler;
    task_cscps_desc.state = cscps_state;
    task_cscps_desc.state_max = CSCPS_STATE_MAX;
    task_cscps_desc.idx_max = CSCPS_IDX_MAX;

    task_desc_register(TASK_CSCPS, task_cscps_desc);
}

#endif //(BLE_CSC_SENSOR)

/// @} CSCPSTASK
