/**
 ****************************************************************************************
 *
 * @file app_paspc.c
 *
 * @brief Application PASPC API
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
 * @addtogroup APP_PASPC_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_PAS_CLIENT
#include "app_paspc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Phone Alert Status Profile Client role - at connection. *//**
 * @param[in] pass   Service structure previously discovered in the database of the peer device. 
 * @param[in] conhdl  Connection handle for which the Alert Notification Status Client role is enabled.
 * @response  PASPC_PASS_CONTENT_IND and PASPC_CMP_EVT
 * @description
 *
 * This API is used by Application to send message to TASK_PASPC for enabling the Collector role 
 * of the Phone Alert Status profile, and it contains the connection handle for the connection  
 * this profile is activated, the connection type and the previously saved discovered PASS details on peer. 
 * 
 * The connection type may be PRF_CON_DISCOVERY (0x00) for discovery/initial configuration or
 * PRF_CON_NORMAL (0x01) for a normal connection with a bonded device. Application shall save
 * those information to reuse them for other connections. During normal connection, previously
 * discovered device information can be reused.
 * 
 * For a normal connection, the response to this request is sent right away after saving the 
 * PASS content in the environment and registering PASPC in GATT to receive the notifications
 * for the known attribute handles in PASS that would be notified.
 * 
 * For a discovery connection, discovery of the peer PASS is started and the response will be
 * sent at the end of the discovery with the discovered attribute details.
 * 
 ****************************************************************************************
 */
void app_paspc_enable_req(struct paspc_pass_content *pass, uint16_t conhdl)
{
    
    struct paspc_enable_cmd * msg = KE_MSG_ALLOC(PASPC_ENABLE_CMD, KE_BUILD_ID(TASK_PASPC, conhdl), TASK_APP,
                                                 paspc_enable_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (pass == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->pass, pass, sizeof(struct paspc_pass_content));
    }

    // Send the message
    ke_msg_send(msg);
}


/*
 ****************************************************************************************
 * @brief Read the value of an attribute in the peer device database. *//**
 * @param[in] read_code     Read code.
 *  - PASPC_RD_ALERT_STATUS
 *  - PASPC_RD_RINGER_SETTING
 *  - PASPC_RD_WR_ALERT_STATUS_CFG
 *  - PASPC_RD_WR_RINGER_SETTING_CFG
 * @param[in] conhdl        Connection handle.
 * @response  PASPC_VALUE_IND and PASPC_CMP_EVT
 * @description
 *
 * This API is used by Application to send a PASPC_READ_CMD with the parameters deduced from 
 * the read_code. Read the value of an attribute in the peer device database.
 *
 ****************************************************************************************
 */
void app_paspc_read_req(uint8_t read_code, uint16_t conhdl)
{
    struct paspc_read_cmd * msg = KE_MSG_ALLOC(PASPC_READ_CMD, KE_BUILD_ID(TASK_PASPC, conhdl), TASK_APP,
                                               paspc_read_cmd);

    ///Connection handle
    msg->conhdl = conhdl;
    /// Read code
    msg->read_code = read_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Write the value of an attribute in the peer device database. *//**
 * @param[in] write_code
 *  - PASPC_WR_RINGER_CTNL_PT
 *  - PASPC_RD_WR_ALERT_STATUS_CFG
 *  - PASPC_RD_WR_RINGER_SETTING_CFG
 * @param[in] value
 *  - Ringer Control Point characteristic value
 *  - Alert Status characteristic client characteristic configuration value
 *  - Ringer Setting characteristic client characteristic configuration value
 * @param[in] conhdl          Connection handle for which the Phone Alert Status Profile
 *                            Client role is enabled.
 * @response  PASPC_CMP_EVT
 * @description
 *
 * This API message shall be used by the application to write the value of one of the
 * writable attribute in the peer device database.
 * 
 ****************************************************************************************
 */
void app_paspc_wr_char_req(uint8_t write_code, union write_value_tag *value, uint16_t conhdl)
{
    struct paspc_write_cmd *msg = KE_MSG_ALLOC(PASPC_WRITE_CMD, KE_BUILD_ID(TASK_PASPC, conhdl), TASK_APP,
                                               paspc_write_cmd);
    ///Write code
    msg->write_code = write_code;
    ///Value
    msg->value = *value;
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

#endif // BLE_PAS_CLIENT

/// @} APP_PASPC_API
