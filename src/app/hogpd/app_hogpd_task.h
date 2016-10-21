/**
 ****************************************************************************************
 *
 * @file app_hogpd_task.h
 *
 * @brief Application HOGPD implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_HOGPD_TASK_H_
#define APP_HOGPD_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_HOGPD_TASK HID Over GATT Device Task API
 * @ingroup APP_HOGPD
 * @brief HID Over GATT Device Task API
 *
 * HID Over GATT Device Task APIs are used to handle the message from HOGPD or APP.
 * @{
 ****************************************************************************************
 */

#if BLE_HID_DEVICE

/// @cond

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_hogpd.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define APP_HOGPD_BOOT_KB_IN_REPORT_TO      100 // 1s
#define APP_HOGPD_BOOT_MOUSE_IN_REPORT_TO   100 // 1s
#define APP_HOGPD_REPORT_TO                 100 // 1S

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
// HID device environment variable
struct app_hogpd_env_tag
{
    // Profile role state: enabled/disabled
    uint8_t enabled;
    struct hogpd_features features[HOGPD_NB_HIDS_INST_MAX];
    uint8_t att_tbl[HOGPD_NB_HIDS_INST_MAX][HOGPD_CHAR_MAX];

    uint8_t proto_mode[HOGPD_NB_HIDS_INST_MAX]; // Current Protocol Mode
    uint8_t hids_nb;
    // Connection handle
    uint16_t conhdl;
    // Notification flow control
    bool ntf_sending;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_hogpd_env_tag *app_hogpd_env;

/// @endcond

/**
 ****************************************************************************************
 * @brief 
 *
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the HOGPD.    
 *
 ****************************************************************************************
 */
int app_hogpd_create_db_cfm_handler(ke_msg_id_t const msgid,
                                    struct hogpd_create_db_cfm *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
                                
/*
 ****************************************************************************************
 * @brief Handles the HOGPD disable indication message from the HOGPD.   
 *
 ****************************************************************************************
 */
int app_hogpd_disable_ind_handler(ke_msg_id_t const msgid,
                                  struct hogpd_disable_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);
                             
/*
 ****************************************************************************************
 * @brief Handles the error indication message from the HOGPD.    
 *
 ****************************************************************************************
 */
int app_hogpd_error_ind_handler(ke_msg_id_t const msgid,
                                struct prf_server_error_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);
                            
/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the HOGPD.   
 *
 ****************************************************************************************
 */
int app_hogpd_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                  struct hogpd_ntf_cfg_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the protocol mode indication message   
 *
 ****************************************************************************************
 */
int app_hogpd_proto_mode_ind_handler(ke_msg_id_t const msgid,
                                     struct hogpd_proto_mode_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the report indication message   
 *
 ****************************************************************************************
 */
int app_hogpd_report_ind_handler(ke_msg_id_t const msgid,
                                 struct hogpd_report_info *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the boot report indication message
 *
 ****************************************************************************************
 */
int app_hogpd_boot_report_ind_handler(ke_msg_id_t const msgid,
                                      struct hogpd_boot_report_info *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);
                                  
/*
 ****************************************************************************************
 * @brief Handles the control point indication message     
 *
 ****************************************************************************************
 */
int app_hogpd_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                  struct hogpd_ctnl_pt_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);
                              
/*
 ****************************************************************************************
 * @brief Handles the notify confirm message  
 *
 ****************************************************************************************
 */
int app_hogpd_ntf_sent_cfm_handler(ke_msg_id_t const msgid,
                                   struct hogpd_ntf_sent_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
               
/// @cond                
/*
 ****************************************************************************************
 * @brief 
 *
 ****************************************************************************************
 */
int app_hogpd_boot_kb_in_report_timer_handler(ke_msg_id_t const msgid,
                                              void *param,
                                              ke_task_id_t const dest_id,
                                              ke_task_id_t const src_id);
                                          
/*
 ****************************************************************************************
 * @brief
 *
 ****************************************************************************************
 */
int app_hogpd_boot_mouse_in_report_timer_handler(ke_msg_id_t const msgid,
                                                 void *param,
                                                 ke_task_id_t const dest_id,
                                                 ke_task_id_t const src_id);
                                  
/*
 ****************************************************************************************
 * @brief
 *
 ****************************************************************************************
 */
int app_hogpd_report_timer_handler(ke_msg_id_t const msgid,
                                   void *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);  
/// @endcond
                                
#endif // BLE_HID_DEVICE

/// @} APP_HOGPD_TASK

#endif // APP_HOGPD_TASK_H_
