/**
 ****************************************************************************************
 *
 * @file app_tips.c
 *
 * @brief Application TIPS API
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
 * @addtogroup APP_TIPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_env.h"

#if BLE_TIP_SERVER
#include "app_tips.h"

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
 * @brief Create the time server database - at initiation       *//**
 *
 * @param[in] features Indicate if optional features are supported or not. Possible bit-mask
 * values are:
 * - TIPS_CTS_LOC_TIME_INFO_SUP
 * - TIPS_CTS_REF_TIME_INFO_SUP
 * - TIPS_NDCS_SUP
 * - TIPS_RTUS_SUP
 *
 * @response TIPS_CREATE_DB_CFM
 * @description
 * This function shall be used to add one instance of the Current Time Service,
 * optionally one instance of the Next DST Change Service and one instance of the Reference
 * Time Update Service.
 ****************************************************************************************
 */
void app_tips_create_db(uint8_t features)
{
    struct tips_create_db_req * msg = KE_MSG_ALLOC(TIPS_CREATE_DB_REQ, TASK_TIPS, TASK_APP, tips_create_db_req);

    msg->features = features;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the time server role - at connection        *//**
 * 
 * @param[in] conhdl Connection handle for which the Time Server role is enabled.
 * @param[in] sec_lvl Security level required for protection of attributes
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * @param[in] con_type Connection type: PRF_CON_DISCOVERY (0x00) , PRF_CON_NORMAL (0x01) 
 * @param[in] current_time_ntf_en Value stored for Current Time Notification Client
 * Configuration Char. 
 *
 * @response None or TIPS_ERROR_IND
 * @description
 * This function is used for enabling the Time Server role of the Time profile.
 ****************************************************************************************
 */
void app_tips_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t current_time_ntf_en)
{
    struct tips_enable_req * msg = KE_MSG_ALLOC(TIPS_ENABLE_REQ, TASK_TIPS, TASK_APP, tips_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->current_time_ntf_en = current_time_ntf_en;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send update current time request - at connection      *//**
 *
 * @param[in] conhdl Connection handle for which the Time Server role is enabled.
 * @param[in] current_time Pointer to the struct tip_curr_time containing Current Time value
 * @param[in] enable_ntf_send Define if a notification of new current time value will be
 * send, 0: Disable, 1: Enable
 * The enable_ntf_send parameter shall be used to be conform with the following CTS
 * Specification requirement:
 * If the time of the Current Time Server is changed because of reference time update, then no
 * notification shall be sent to Current Time Service Client within the 15 minutes from the last
 * notification, unless one of both of the two statements below are true:
 *    - The new time information differs by more than 1 minute from the Current Time Server
 *      time previous to the update.
 *    - The update was caused by the client (interacting with another service).
 *
 * @response None or TIPS_ERROR_IND_SEND
 * @description
 * This function is used by the application for requesting an update of the Current Time
 * characteristic value.
 ****************************************************************************************
 */
void app_tips_upd_curr_time_req(uint16_t conhdl, struct tip_curr_time *current_time, uint8_t enable_ntf_send)
{
    struct tips_upd_curr_time_req * msg = KE_MSG_ALLOC(TIPS_UPD_CURR_TIME_REQ, TASK_TIPS, TASK_APP,
                                                       tips_upd_curr_time_req);
    
    msg->conhdl = conhdl;
    msg->current_time = *current_time;
    msg->enable_ntf_send = enable_ntf_send;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send update local time info request - at connection       *//**
 *
 * @param[in] conhdl Connection handle for which the Time Server role is enabled
 * @param[in] loc_time_info Pointer to the struct tip_loc_time_info containing Local Time Information
 *
 * @response None or TIPS_ERROR_IND_SEND
 * @description
 * This function is used by the application for updating the Local Time Information
 * Characteristic value.
 ****************************************************************************************
 */
void app_tips_upd_local_time_info_req(uint16_t conhdl, struct tip_loc_time_info *loc_time_info)
{
    struct tips_upd_local_time_info_req * msg = KE_MSG_ALLOC(TIPS_UPD_LOCAL_TIME_INFO_REQ, TASK_TIPS, TASK_APP,
                                                             tips_upd_local_time_info_req);
    
    msg->conhdl = conhdl;
    msg->loc_time_info = *loc_time_info;
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send update reference time info request - at connection       *//**
 *
 * @param[in] conhdl Connection handle for which the Time Server role is enabled
 * @param[in] ref_time_info Pointer to the struct tip_ref_time_info containing Reference Time Information
 *
 * @response None or TIPS_ERROR_IND_SEND
 * @description
 * This function is used by the application for updating the Reference Time Information
 * Characteristic value.
 ****************************************************************************************
 */
void app_tips_upd_ref_time_info_req(uint16_t conhdl, struct tip_ref_time_info *ref_time_info)
{
    struct tips_upd_ref_time_info_req * msg = KE_MSG_ALLOC(TIPS_UPD_REF_TIME_INFO_REQ, TASK_TIPS, TASK_APP,
                                                           tips_upd_ref_time_info_req);
                                                             
    msg->conhdl = conhdl;
    msg->ref_time_info = *ref_time_info;                                     
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send update time DST request - at connection      *//**
 *
 * @param[in] conhdl Connection handle for which the Time Server role is enabled
 * @param[in] time_with_dst Pointer to the struct tip_time_with_dst containing Time With DST
 *
 * @response None or TIPS_ERROR_IND_SEND
 * @description
 * This function is used by the application for updating the Reference Time Information
 * Characteristic value.
 ****************************************************************************************
 */
void app_tips_upd_time_dst_req(uint16_t conhdl, struct tip_time_with_dst *time_with_dst)
{
    struct tips_upd_time_dst_req * msg = KE_MSG_ALLOC(TIPS_UPD_TIME_DST_REQ, TASK_TIPS, TASK_APP,
                                                      tips_upd_time_dst_req);
                                         
    msg->conhdl = conhdl;
    msg->time_with_dst = *time_with_dst;
    ke_msg_send(msg);                                                 
}

/*
 ****************************************************************************************
 * @brief Send update time update state request - at connection     *//**
 *
 * @param[in] conhdl Connection handle for which the Time Server role is enabled
 * @param[in] time_upd_state Pointer to the struct tip_time_upd_state containing Time Update State
 *
 * @response None or TIPS_ERROR_IND_SEND
 * @description
 * This function is used by the application for updating the Reference Time Information
 * Characteristic value.
 ****************************************************************************************
 */
void app_tips_upd_time_upd_state_req(uint16_t conhdl, struct tip_time_upd_state *time_upd_state)
{
    struct tips_upd_time_upd_state_req * msg = KE_MSG_ALLOC(TIPS_UPD_TIME_UPD_STATE_REQ, TASK_TIPS, TASK_APP,
                                                            tips_upd_time_upd_state_req);
                                         
    msg->conhdl = conhdl;
    msg->time_upd_state = *time_upd_state;
    ke_msg_send(msg);
}

#endif // BLE_TIP_SERVER

/// @} APP_TIPS_API
