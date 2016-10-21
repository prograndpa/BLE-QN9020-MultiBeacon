/**
 ****************************************************************************************
 *
 * @file app_glpc.c
 *
 * @brief Application GLPC API
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
 * @addtogroup APP_GLPC_API 
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_GL_COLLECTOR
#include "app_glpc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Glucose Profile Collector - at connection.     *//**
 *
 * @param[in] gls      Existing handle values GLS (see Glucose Content Structure (struct gls_content))
 * @param[in] conhdl   Connection handle for which the profile Glucose collector role is enabled
 * @response  GLPC_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Collector role of the Glucose profile. This function 
 *  contains BLE connection handle, the connection type and the previously saved discovered GLS details on peer. 
 *  The connection type may be 0 = Connection for discovery/initial configuration or 1 = Normal connection. This 
 *  parameter is used by Application to discover peer device services once at first connection. Application shall save those 
 *  information to reuse them for other connections. During normal connection, previously discovered device 
 *  information can be reused.
 *
 *  This is useful since most use cases allow Glucose sensor to disconnect the link once all measurements have been sent 
 *  to Collector.
 *
 *  If it is a discovery/configuration type of connection, the GLS parameters are useless, and they will be filled with 0's. 
 *  Otherwise they will contain pertinent data which will be kept in the Collector environment while enabled. It allows for 
 *  the Application to not be aware of attribute details. 
 *
 *  For a normal connection, the response to this request is sent right away after saving the GLS content in the 
 *  environment and registering GLPC in GATT to receive the indications and notifications for the known attribute handles
 *  in GLS that would be notified/indicated. For a discovery connection, discovery of the peer GLS is started and the 
 *  response will be sent at the end of the discovery with the discovered attribute details. 
 *
 ****************************************************************************************
 */
void app_glpc_enable_req(struct gls_content *gls, uint16_t conhdl)
{
    struct glpc_enable_req * msg = KE_MSG_ALLOC(GLPC_ENABLE_REQ, KE_BUILD_ID(TASK_GLPC, conhdl), TASK_APP,
                                                 glpc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (gls == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->gls, gls, sizeof(struct gls_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Register to Glucose sensor notifications - request.    *//**
 * @param[in] meas_ctx_en   Register or not Glucose measurement context notifications
 * @param[in] conhdl        Connection handle for which the profile Glucose collector role is enabled
 * @response  GLPC_REGISTER_CFM
 * @description
 *
 *  This API is used by the application to register to Glucose sensor notifications and indications.
 *  According to peer available characteristics, it performs in one action all event registration. This shall be performed after
 *  enabling collector first time Glucose sensor is used. This registration shall be kept by peer device if bonding procedure
 *  has been performed.
 *
 *  This procedure shall be done before doing any Record Access Control Point requests.
 *
 ****************************************************************************************
 */
void app_glpc_register_req(bool meas_ctx_en, uint16_t conhdl)
{
    struct glpc_register_req *msg = KE_MSG_ALLOC(GLPC_REGISTER_REQ, KE_BUILD_ID(TASK_GLPC, conhdl), TASK_APP,
                                                glpc_register_req);
    ///Register or not Glucose measurement context notifications
    msg->meas_ctx_en = meas_ctx_en;
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read Glucose sensor features - request.    *//**
 * @param[in] conhdl        Connection handle for which the profile Glucose collector role is enabled
 * @response  GLPC_READ_FEATURES_RSP
 * @description
 *
 *  This API is used by the application to read peer Glucose sensor features.
 ****************************************************************************************
 */
void app_glpc_read_features_req(uint16_t conhdl)
{
    struct glpc_read_features_req *msg = KE_MSG_ALLOC(GLPC_READ_FEATURES_REQ, KE_BUILD_ID(TASK_GLPC, conhdl), TASK_APP,
                                                glpc_read_features_req);
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to record Access Control Point - Request.     *//**
 * @param[in] racp          Record Access Control Point (RACP) Request (struct glp_racp_req)
 * @param[in] conhdl        Connection handle for which the profile Glucose collector role is enabled
 * @response  GLPC_RACP_RSP 
 * @description
 *
 *  This API is used by Application to request execution of a RACP Request on peer Glucose sensor.
 *  This action could be report glucose measurements, report number of measurement, delete measurements or abort 
 *  an on-going operation (see Record Access Control Point (RACP) OP Code). This action contains a filter describing 
 *  which glucose measurement are concerned by the operation.
 *
 *  Possible OP Code: 
 *  - GLP_REQ_REP_STRD_RECS: Report stored records
 *  - GLP_REQ_REP_NUM_OF_STRD_RECS: Report number of stored records
 *  - GLP_REQ_DEL_STRD_RECS: Delete stored records
 *  - GLP_REQ_ABORT_OP: Abort on-going operation.
 *
 *  Possible Operator: 
 *  - GLP_OP_ALL_RECS
 *  - GLP_OP_LT_OR_EQ
 *  - GLP_OP_GT_OR_EQ
 *  - GLP_OP_WITHIN_RANGE_OF
 *  - GLP_OP_FIRST_REC
 *  - GLP_OP_LAST_REC
 *  
 *  Possible filter type: 
 *  - GLP_FILTER_SEQ_NUMBER
 *  - GLP_FILTER_USER_FACING_TIME
 *
 * @note  
 *
 *  During an on-going operation, any other request from collector shall be refused by Glucose service, except 
 *  GLP_REQ_ABORT_OP (Abort operation). In that case on-going operation shall be stopped by glucose sensor. RACP 
 *  response message shall be received from peer Glucose sensor with GLP_REQ_ABORT_OP op_code and status equals 
 *  GLP_RSP_SUCCESS.
 *
 ****************************************************************************************
 */
void app_glpc_racp_req(struct glp_racp_req *racp, uint16_t conhdl)
{
    struct glpc_racp_req *msg = KE_MSG_ALLOC(GLPC_RACP_REQ, KE_BUILD_ID(TASK_GLPC, conhdl), TASK_APP,
                                                glpc_racp_req);

    ///Connection handle
    msg->conhdl = conhdl;
    /// Glucose access control OP Code
    msg->racp_req.op_code = racp->op_code;
    /// Glucose access control Operator
    msg->racp_req.filter.operator = racp->filter.operator;
    /// Glucose Operand Filter Type
    msg->racp_req.filter.filter_type = racp->filter.filter_type;
    /// record access control filter type
    if (msg->racp_req.filter.filter_type == GLP_FILTER_SEQ_NUMBER)
    {
        /// Sequence number filtering (filter_type = GLP_FILTER_SEQ_NUMBER)
        msg->racp_req.filter.val.seq_num = racp->filter.val.seq_num;
    }
    else
    {
        /// User facing time filtering (filter_type = GLP_FILTER_USER_FACING_TIME)
        msg->racp_req.filter.val.time = racp->filter.val.time;
    }
    
    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_GLPC_API
