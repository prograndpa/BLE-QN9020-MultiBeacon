/**
 ****************************************************************************************
 *
 * @file app_proxm.c
 *
 * @brief Application PROXM API
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
 * @addtogroup APP_PROXM_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_PROX_MONITOR
#include "app_proxm.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the Proximity Monitor profile - at connection. *//**
 * @param[in] lls           Link Loss Service information.
 * @param[in] ias           Immediate Alert Service information.
 * @param[in] txps          Tx Power Service information.
 * @param[in] conhdl        Connection handle for which the profile Monitor role is enabled.
 * @response  PROXM_ENABLE_CFM or PROXM_ERROR_IND
 * @description
 *
 *  The API is used for enabling the Monitor role of the Proximity profile. 
 *  This function contains the connection handle for the connection this profile is 
 *  activated, the connection type and the previously saved discovered LLS, IAS and TPS details 
 *  on peer. 
 *
 *  The connection type may be 0 = Connection for discovery or 1 = Normal connection. This 
 *  difference has been made and Application would handle it in order to not discover the 
 *  attributes on the Reporter at every connection, but do it only once and keep the discovered 
 *  details in the Monitor device between connections. 
 *
 *  If it is a discovery type connection, the LLS, IAS and TPS parameters are useless, they will be 
 *  filled with 0's. Otherwise it will contain pertinent data which will be kept in the Monitor 
 *  environment while enabled. It allows for the Application to not be aware of attribute details, 
 *  and only give them to the profile role from a storage area where they are kept in between 
 *  connections (NVDS...). 
 *
 *  For a normal connection, the response to this request is sent right away after saving the lls, ias 
 *  and tps content in the environment. For a  discovery  connection, discovery of the peer 
 *  attributes is started and the response will be sent at the end of the discovery with the 
 *  discovered attribute details. The discovery starts with the 3 services, then with the 
 *  characteristics of the discovered services. If an error happens during discovery (mandatory 
 *  conditions present in the specification are not respected - missing services, mandatory 
 *  characteristics...) it is signaled right away to the application setting an appropriate value in the 
 *  status parameter of the response.
 *
 ****************************************************************************************
 */
void app_proxm_enable_req(struct svc_content *lls, 
                          struct svc_content *ias,
                          struct svc_content *txps,
                          uint16_t conhdl)
{
    struct proxm_enable_req * msg = KE_MSG_ALLOC(PROXM_ENABLE_REQ, KE_BUILD_ID(TASK_PROXM, conhdl), TASK_APP,
                                                 proxm_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (lls == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->lls, lls, sizeof(struct svc_content));
        if (txps != NULL)
        {
            memcpy(&msg->ias, ias, sizeof(struct svc_content));
            memcpy(&msg->txps, txps, sizeof(struct svc_content));
        }
    }
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to Read LLS Alert level (for IAS not allowed). *//**
 * @param[in] conhdl        Connection handle for which the profile Monitor role is enabled.
 * @response  PROXM_RD_CHAR_RSP or PROXM_ERROR_IND
 * @description
 *
 *  This API is used for reading the alert level in LLS Alert Level Characteristic.  
 *  Upon reception, the Monitor checks if a valid handle for this characteristic is 
 *  known in the monitor environment, otherwise sending a PROXM_ERROR_IND with error code 
 *  0x02. If the characteristic handle in the environment is valid, a read characteristic request is 
 *  built for TASK_GATT using the LLS service and characteristic handle values. When the read 
 *  response is received from GATT, it is sent to the Application to be analyzed.
 *
 ****************************************************************************************
 */
void app_proxm_rd_alert_lvl_req(uint16_t conhdl)
{
    struct proxm_rd_alert_lvl_req *msg = KE_MSG_ALLOC(PROXM_RD_ALERT_LVL_REQ, KE_BUILD_ID(TASK_PROXM, conhdl), TASK_APP,
                                                proxm_rd_alert_lvl_req);
    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to Set Alert level. *//**
 * @param[in] svc_code      code for the service in which the alert level should be written.
 *  - PROXM_SET_LK_LOSS_ALERT  ///Code for LLS Alert Level Char.
 *  - PROXM_SET_IMMDT_ALERT    ///Code for IAS Alert Level Char.
 * @param[in] lvl           Alert level.
 *  - PROXM_ALERT_NONE
 *  - PROXM_ALERT_MILD
 *  - PROXM_ALERT_HIGH
 * @param[in] conhdl        Connection handle for which the profile Monitor role is enabled.
 * @response  PROXM_WR_CHAR_RSP or PROXM_ERROR_IND
 * @description
 *
 *  This API is used by the application to set either a LLS Alert Level or an 
 *  IAS Alert Level. Since these two service have characteristics of the same type (but not the 
 *  same properties - LLS one is R&W, IAS one is Write no Response only), one API message for a 
 *  very similar purpose was considered sufficient, and therefore a simple byte for differentiating 
 *  whether the alert code should be set in LLS or IAS characteristic is used.
 *
 *  Upon reception of this request, the connection handle is checked, then the alert level to set in 
 *  order to ensure a valid value, and then the svc_code for 0 or 1.. In case any of these checks 
 *  fail, a PROXM_ERROR_IND message is sent to the Application with error code 0x01.
 *
 *  If the checks are successful, a write characteristic request is build for TASK_GATT, with the 
 *  appropriate type (Simple Write or Write no response) and using the characteristic handle for 
 *  the service indicated by svc_code. When the Write Response is received from TASK_GATT, it is 
 *  sent to the Application for it to check the status.
 *
 ****************************************************************************************
 */
void app_proxm_wr_alert_lvl_req(uint8_t svc_code, uint8_t lvl, uint16_t conhdl)
{
    struct proxm_wr_alert_lvl_req *msg = KE_MSG_ALLOC(PROXM_WR_ALERT_LVL_REQ, KE_BUILD_ID(TASK_PROXM, conhdl), TASK_APP,
                                                proxm_wr_alert_lvl_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///0=LLS or 1=IAS, code for the service in which the alert level should be written
    msg->svc_code = svc_code;
    ///Alert level
    msg->lvl = lvl;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to Read TX Power Level. *//**
 * @param[in] conhdl        Connection handle for which the profile Monitor role is enabled.
 * @response  PROXM_RD_CHAR_RSP or PROXM_ERROR_IND
 * @description
 *
 *  This API is used for reading the tx power level in TPS Tx Power Level Characteristic. 
 *  Upon reception, the Monitor checks if a valid handle for this characteristic is 
 *  known in the monitor environment, otherwise sending a PROXM_ERROR_IND with error code 
 *  0x02. If the characteristic handle in the environment is valid, a read characteristic request is 
 *  built for TASK_GATT using the TPS service and characteristic handle values. When the read 
 *  response is received from GATT, it is sent to the Application to be analyzed.
 *
 ****************************************************************************************
 */
void app_proxm_rd_txpw_lvl_req(uint16_t conhdl)
{
    // Put the device in general discoverable and undirected connectable mode
    struct proxm_rd_txpw_lvl_req *msg = KE_MSG_ALLOC(PROXM_RD_TXPW_LVL_REQ, KE_BUILD_ID(TASK_PROXM, conhdl), TASK_APP,
                                                proxm_rd_txpw_lvl_req);

    ///Connection handle
    msg->conhdl = conhdl;

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_PROXM_API
