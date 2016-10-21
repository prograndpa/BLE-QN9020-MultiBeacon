/**
 ****************************************************************************************
 *
 * @file app_hrpc.c
 *
 * @brief Application HRPC API
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
 * @addtogroup APP_HRPC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_HR_COLLECTOR
#include "app_hrpc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Heart Rate Collector profile - at connection. *//**
 * @param[in] hrs           Heart Rate Service Content Structure.
 * @param[in] conhdl        Connection handle for which the profile Heart Rate Collector role is enabled.
 * @response  HRPC_ENABLE_CFM
 * @description
 *
 *  This API is used for enabling the Collector role of the Heart Rate profile. This function contains 
 *  BLE connection handle, the connection type and the previously saved discovered HRS details on peer. 
 *  The connection type may be 0 = Connection for discovery/initial configuration or 1 = Normal connection. This 
 *  parameter is used by Application to discover peer device services once at first connection. Application shall save those 
 *  information to reuse them for other connections. During normal connection, previously discovered device 
 *  information can be reused.
 *
 *  This is useful since most use cases allow Heart Rate Sensor to disconnect the link once all measurements have been 
 *  sent to Collector.
 *
 *  If it is a discovery /configuration type of connection, the HRS parameters are useless, they will be filled with 0's. 
 *  Otherwise they will contain pertinent data which will be kept in the Collector environment while enabled. It allows for 
 *  the Application to not be aware of attribute details.
 *
 *  For a normal connection, the response to this request is sent right away after saving the HRS content in the 
 *  environment and registering HRPC in GATT to receive the indications and notifications for the known attribute 
 *  handles in HRS that would be notified/indicated. For a discovery connection, discovery of the peer HRS is started and 
 *  the response will be sent at the end of the discovery with the discovered attribute details. 
 *
 ****************************************************************************************
 */
void app_hrpc_enable_req(struct hrs_content *hrs, uint16_t conhdl)
{
    struct hrpc_enable_req * msg = KE_MSG_ALLOC(HRPC_ENABLE_REQ, KE_BUILD_ID(TASK_HRPC, conhdl), TASK_APP,
                                                 hrpc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (hrs == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->hrs, hrs, sizeof(struct hrs_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a HRS characteristic value. *//**
 * @param[in] char_code     Code for which characteristic to read:
 *  - HRPC_RD_HRS_HR_MEAS          ///Read HRS Heart Rate Measurement
 *  - HRPC_RD_HRS_BODY_SENSOR_LOC  ///Body Sensor Location
 *  - HRPC_RD_HRS_CNTL_POINT       ///Heart Rate Control Point
 *  - HRPC_RD_HRS_HR_MEAS_CFG      ///Read HRS Heart Rate Measurement Client Cfg. Desc
 * @param[in] conhdl        Connection handle for which the profile Heart Rate Collector role is enabled.
 * @response  HRPC_RD_CHAR_RSP or HRPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_READ_CHAR_REQ with the parameters 
 *  deduced from the char_code. The definitions for the different mapping codes for characteristics that are possibly 
 *  readable are in hrpc.h (for HRS). Upon reception of this message, HRPC checks whether the parameters are correct, 
 *  then if the handle for the characteristic is valid (not 0x0000) and the request is sent to GATT. When the peer has 
 *  responded to GATT, and the response is routed to HRPC, the HRPC_RD_CHAR_RSP message will be generically built 
 *  and the Application must be able to interpret it based on the read request it made. And error status is also possible 
 *  either for the Read procedure or for the application request, in the second case, the HRPC_ERROR_IND message is 
 *  sent to Application.
 *
 ****************************************************************************************
 */
void app_hrpc_rd_char_req(uint8_t char_code, uint16_t conhdl)
{
    struct hrpc_rd_char_req *msg = KE_MSG_ALLOC(HRPC_RD_CHAR_REQ, KE_BUILD_ID(TASK_HRPC, conhdl), TASK_APP,
                                                hrpc_rd_char_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///Characteristic value code
    msg->char_code = char_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 2 characteristics that can be handled. *//**
 * @param[in] cfg_val       Stop/notify/indicate value to configure into the peer characteristic.
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] conhdl        Connection handle for which the profile Heart Rate Collector role is enabled.
 * @response  HRPC_WR_CHAR_RSP or HRPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to send a GATT_WRITE_CHAR_REQ with the parameters 
 *  deduced from the char_code and cfg_val. The definitions for the different codes for characteristics that can be 
 *  configured to indicate/notify are in hrpc.h. Upon reception of this message, HRPC checks whether the parameters are 
 *  correct, then if the handle for the characteristic is valid (not 0x0000) and the request is sent to GATT. When the peer 
 *  has responded to GATT, and the response is routed to HRPC, the HRPC_WR_CHAR_RSP message will be generically 
 *  built and sent to Application. An error status is also possible either for the Write procedure or for the application 
 *  request, in the second case, the HRPC_ERROR_IND message is sent to Application. 
 *
 ****************************************************************************************
 */
void app_hrpc_cfg_indntf_req(uint16_t cfg_val, uint16_t conhdl)
{
    struct hrpc_cfg_indntf_req *msg = KE_MSG_ALLOC(HRPC_CFG_INDNTF_REQ, KE_BUILD_ID(TASK_HRPC, conhdl), TASK_APP,
                                                hrpc_cfg_indntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    msg->cfg_val = cfg_val;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief APP request for control point write (to reset Energy Expanded). *//**
 * @param[in] val           Reset(1).
 * @param[in] conhdl        Connection handle for which the profile Heart Rate Collector role is enabled.
 * @response  HRPC_WR_CHAR_RSP or HRPC_ERROR_IND
 * @description
 *
 *  This API is used by the application to write control point attribute in order to reset Energy 
 *  Expanded value.
 *
 ****************************************************************************************
 */
void app_hrpc_wr_cntl_point_req(uint8_t val, uint16_t conhdl)
{
    struct hrpc_wr_cntl_point_req *msg = KE_MSG_ALLOC(HRPC_WR_CNTL_POINT_REQ, KE_BUILD_ID(TASK_HRPC, conhdl), TASK_APP,
                                                hrpc_wr_cntl_point_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///value
    msg->val = val;

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_HRPC_API
