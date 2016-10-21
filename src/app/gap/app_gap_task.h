/**
 ****************************************************************************************
 *
 * @file app_gap_task.h
 *
 * @brief Application GAP task implementation
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_GAP_TASK_H_
#define APP_GAP_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP_GAP_TASK Generic Access Profile Task API
 * @ingroup APP_GAP
 * @brief Generic Access Profile Task API
 *
 * GAP Task APIs are used to handle the message from GAP or APP.
 * @{
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles ready indication from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_ready_evt_handler(ke_msg_id_t const msgid, void const *param,
                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles device reset complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_reset_req_cmp_handler(ke_msg_id_t const msgid, struct gap_reset_req_cmp_evt const *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles set device name complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_devname_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles set security request complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_sec_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_set_sec_req_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles read version request complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_read_ver_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_read_ver_req_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles read bdaddr request complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_read_bdaddr_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_read_bdaddr_req_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles inquiry result from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_dev_inq_result_handler(ke_msg_id_t const msgid,
                           struct gap_dev_inq_result_evt const *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles inquiry complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_dev_inq_cmp_handler(ke_msg_id_t const msgid,
                           struct gap_event_common_cmd_complete const *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles scan complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_dev_scan_result_handler(ke_msg_id_t const msgid,
                                   struct gap_adv_report_evt const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
/*
 ****************************************************************************************
 * @brief Handles inquiry cancel complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_scan_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles set mode complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_mode_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles advertising request complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_adv_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles create connection request complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_le_create_conn_req_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_le_create_conn_req_cmp_evt const *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                       
/*
 ****************************************************************************************
 * @brief Handles cancel create connection complete event from the GAP.    *//**
 *
 * @param[in] msgid     GAP_CANCEL_CONN_REQ_CMP_EVT
 * @param[in] param     NULL
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GAP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application the outcome of cancel connection.
 ****************************************************************************************
 */
int app_gap_cancel_conn_req_cmp_evt_handler(ke_msg_id_t const msgid, void const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_discon_cmp_evt_handler(ke_msg_id_t const msgid, struct gap_discon_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles name request complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_name_req_cmp_handler(ke_msg_id_t const msgid, struct gap_name_req_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
                            
/**
 ****************************************************************************************
 * @brief Handles read the white list size of the local device complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_le_rd_wlst_size_cmd_cmp_handler(ke_msg_id_t const msgid, struct gap_rd_wlst_size_cmd_complete const *param,
                                            ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                            
/**
 ****************************************************************************************
 * @brief Handles add device to white list complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_le_add_dev_to_wlst_req_cmp_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                            
/**
 ****************************************************************************************
 * @brief Handles remove device from white list complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_le_rmv_dev_from_wlst_req_cmp_handler(ke_msg_id_t const msgid, struct gap_event_common_cmd_complete const *param,
                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                            
/**
 ****************************************************************************************
 * @brief Handles read remote features complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_le_rd_remote_feat_req_cmp_handler(ke_msg_id_t const msgid, struct gap_le_rd_remote_feat_req_cmp_evt const *param,
                                              ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                              
/**
 ****************************************************************************************
 * @brief Handles read remote version information complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_rd_remote_ver_info_cmp_handler(ke_msg_id_t const msgid, struct gap_rd_rem_ver_info_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                           
/**
 ****************************************************************************************
 * @brief Handles set random address complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_random_add_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_random_addr_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                           
/**
 ****************************************************************************************
 * @brief Handles param update response from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_param_update_resp_handler(ke_msg_id_t const msgid, struct gap_param_update_resp const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                      
/**
 ****************************************************************************************
 * @brief Handles change param request complete from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_change_param_req_cmp_handler(ke_msg_id_t const msgid, struct gap_change_param_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                           
/**
 ****************************************************************************************
 * @brief Handles set reconnection address complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_recon_addr_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_recon_addr_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                           
/**
 ****************************************************************************************
 * @brief Handles set the privacy settings of the peer peripheral device complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_ph_privacy_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_ph_privacy_req_cmp_evt const *param,
                                           ke_task_id_t const dest_id, ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles set privacy feature of the local device complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_set_privacy_req_cmp_handler(ke_msg_id_t const msgid, struct gap_set_privacy_req_cmp_evt const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                        
/**
 ****************************************************************************************
 * @brief Handles channel map update operation complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_channel_map_cmp_handler(ke_msg_id_t const msgid, struct gap_channel_map_cmp_evt const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles read RSSI value complete event from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_read_rssi_req_cmp_handler(ke_msg_id_t const msgid, struct gap_read_rssi_req_cmp_evt const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                      
/*
 ****************************************************************************************
 * @brief Handles parameter update request indication from the slave.
 *
 ****************************************************************************************
 */
int app_gap_param_update_req_ind_handler(ke_msg_id_t const msgid, struct gap_param_update_req_ind const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles bond request complete indication from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_bond_req_cmp_ind_handler(ke_msg_id_t const msgid, struct gap_bond_req_cmp_evt const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles bond request indication from the GAP.
 *
 ****************************************************************************************
 */
int app_gap_bond_req_ind_handler(ke_msg_id_t const msgid, struct gap_bond_req_ind const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
                               
/*
 ****************************************************************************************
 * @brief Handles the GATT resource access request. 
 *
 ****************************************************************************************
 */
int app_gatt_resource_access_req_handler(ke_msg_id_t const msgid, struct gatt_resource_access_req const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id);

/// @} APP_GAP_TASK

#endif // APP_GAP_TASK_H_
