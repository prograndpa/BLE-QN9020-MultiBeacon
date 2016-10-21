/**
 ****************************************************************************************
 *
 * @file app_rscpc.c
 *
 * @brief Application RSCPC API
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
 * @addtogroup APP_RSCPC_API
 * @{
 ****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_RSC_COLLECTOR
#include "app_rscpc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Running Speed and Cadence profile Collector - at connection. *//**
 * @param[in] rscs          Service structure previously discovered in the database of the peer device.
 * @param[in] conhdl        Connection handle for which the profile Collector role is enabled.
 * @response  RSCPC_RSCS_CONTENT_IND and RSCPC_CMP_EVT
 * @description
 *
 * This API is used by Application to send message to TASK_RSCPC for enabling the Collector role 
 * of the Running Speed and Cadence profile, and it contains the connection handle for the connection  
 * this profile is activated, the connection type and the previously saved discovered RSCS details on peer. 
 * 
 * The connection type may be PRF_CON_DISCOVERY(0x00) for discovery/initial configuration or PRF_CON_NORMAL(0x01)
 * for a normal connection with a bonded device. Application shall save those information to reuse them for
 * others connections. During normal connection, previously discovered information can ben reused.
 *
 * For a normal connection, the response to this request is sent right away after saving the RSCS 
 * content in the environment and registering RSCSC in GATT to receive the notifications for the  
 * known attribute handles in RSCS that would be notified. For a discovery connection, 
 * discovery of the peer RSCS is started and the response will be sent at the end of the 
 * discovery with the discovered attribute details.
 *
 ****************************************************************************************
 */
void app_rscpc_enable_req(struct rscpc_rscs_content *rscs, uint16_t conhdl)
{
    struct rscpc_enable_cmd * msg = KE_MSG_ALLOC(RSCPC_ENABLE_CMD, KE_BUILD_ID(TASK_RSCPC, conhdl), TASK_APP,
                                                 rscpc_enable_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (rscs == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->rscs, rscs, sizeof(struct rscpc_rscs_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Read the value of an attribute in the peer device database. *//**
 * @param[in] read_code     Read code.
 *  - RSCPC_RD_RSC_FEAT
 *  - RSCPC_RD_SENSOR_LOC
 *  - RSCPC_RD_WR_RSC_MEAS_CFG
 *  - RSCPC_RD_WR_SC_CTNL_PT_CFG
 * @param[in] conhdl        Connection handle.
 * @response  RSCPC_VALUE_IND or RSCPC_CMP_EVT
 * @description
 *
 * This API is used by Application to send a RSCPC_READ_CMD with the parameters deduced from 
 * the read_code. Read the value of an attribute in the peer device database.
 *
 ****************************************************************************************
 */
void app_rscpc_read_req(uint8_t read_code, uint16_t conhdl)
{
    struct rscpc_read_cmd * msg = KE_MSG_ALLOC(RSCPC_READ_CMD, KE_BUILD_ID(TASK_RSCPC, conhdl), TASK_APP,
                                               rscpc_read_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    /// Read code
    msg->read_code = read_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Configure sending of notification/indication in the peer device database. *//**
 * @param[in] desc_code     Descriptor code. Possible values are:
 *  - RSCPC_RD_WR_RSC_MEAS_CFG
 *  - RSCPC_RD_WR_SC_CTNL_PT_CFG
 * @param[in] ntfind_cfg    NTF/IND configuration. Possible values for setting client configuration characteristics
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] conhdl        Connection handle.
 * @response  RSCPC_CMP_EVT
 * @description
 *
 * This API is used by Application to configure sending of notification/indication in the peer
 * device database. If desc_code sets RSCPC_RD_WR_RSC_MEAS_CFG, parameter ntfind_cfg sets
 * PRF_CLI_STOP_NTFIND or PRF_CLI_START_NTF, desc_code sets RSCPC_RD_WR_SC_CTNL_PT_CFG,
 * parameter ntfind_cfg sets PRF_CLI_STOP_NTFIND or PRF_CLI_START_IND.
 *
 ****************************************************************************************
 */
void app_rscpc_cfg_ntfind_req(uint8_t desc_code, uint16_t ntfind_cfg, uint16_t conhdl)
{
    struct rscpc_cfg_ntfind_cmd * msg = KE_MSG_ALLOC(RSCPC_CFG_NTFIND_CMD, KE_BUILD_ID(TASK_RSCPC, conhdl), TASK_APP,
                                                     rscpc_cfg_ntfind_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Ntf/Ind Configuration
    msg->ntfind_cfg = ntfind_cfg;
    ///Descriptor code
    msg->desc_code = desc_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send the value of the SC Control Point. *//**
 * @param[in] sc_ctnl_pt    SC Control Point Request.
 * @param[in] conhdl        Connection handle.
 *
 * @response  RSCPC_CMP_EVT
 * @description
 *
 * This API is used by Application to write the value of the SC Control Point characteristic.
 * If the Control Point characteristic has not beem found in the peer device database during
 * the discovery procedure, a RSCPC_CMP_EVT message is sent back to the requester with a 
 * PRF_ERR_INVALID_HDL error status.
 ****************************************************************************************
 */
void app_rscpc_ctnl_pt_cfg_req(struct rscp_sc_ctnl_pt_req *sc_ctnl_pt, uint16_t conhdl)
{
    struct rscpc_ctnl_pt_cfg_cmd * msg = KE_MSG_ALLOC(RSCPC_CTNL_PT_CFG_CMD, KE_BUILD_ID(TASK_RSCPC, conhdl), TASK_APP,
                                                      rscpc_ctnl_pt_cfg_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    ///SC Control Point Request
    msg->sc_ctnl_pt = *sc_ctnl_pt;

    //Send the message
    ke_msg_send(msg);
}

#endif // BLE_RSC_COLLECTOR

/// @} APP_RSCPC_API
